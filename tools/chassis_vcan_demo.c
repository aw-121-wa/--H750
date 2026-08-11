#include "control/chassis/chassis_kinematics.h"
#include "devices/motor/zdt_x42s/zdt_x42s_protocol.h"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#define ZDT_ACCELERATION 0U

static int parse_float(const char *text, float *value)
{
    char *end = NULL;
    float parsed;

    errno = 0;
    parsed = strtof(text, &end);
    if ((errno != 0) || (end == text) || (*end != '\0'))
    {
        return -1;
    }

    *value = parsed;
    return 0;
}

static int open_can_socket(const char *if_name)
{
    struct ifreq ifr;
    struct sockaddr_can addr;
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (fd < 0)
    {
        perror("socket");
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1U);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl SIOCGIFINDEX");
        close(fd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(fd);
        return -1;
    }

    return fd;
}

static int send_frame(int fd, uint32_t id, const uint8_t *payload, size_t len)
{
    struct can_frame frame;
    ssize_t written;

    if ((payload == NULL) || (len > CAN_MAX_DLEN))
    {
        return -1;
    }

    memset(&frame, 0, sizeof(frame));
    frame.can_id = CAN_EFF_FLAG | id;
    frame.can_dlc = (uint8_t)len;
    memcpy(frame.data, payload, len);

    written = write(fd, &frame, sizeof(frame));
    if (written != (ssize_t)sizeof(frame))
    {
        perror("write");
        return -1;
    }

    return 0;
}

static void print_frame(uint32_t id, const uint8_t *payload, size_t len)
{
    printf("%08X#", id);
    for (size_t i = 0U; i < len; ++i)
    {
        printf("%02X", payload[i]);
    }
    printf("\n");
}

static void print_usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [ifname] [vy] [vx] [vw] [fine]\n"
            "Example: %s vcan0 20 0 0 0\n",
            program,
            program);
}

int main(int argc, char **argv)
{
    const char *if_name = argc > 1 ? argv[1] : "vcan0";
    float vy = 20.0f;
    float vx = 0.0f;
    float vw = 0.0f;
    bool fine = false;
    float wheel_rpm[CHASSIS_WHEEL_COUNT];
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH];
    int fd;

    if (((argc > 2) && (parse_float(argv[2], &vy) != 0)) ||
        ((argc > 3) && (parse_float(argv[3], &vx) != 0)) ||
        ((argc > 4) && (parse_float(argv[4], &vw) != 0)))
    {
        print_usage(argv[0]);
        return 2;
    }
    if (argc > 5)
    {
        fine = atoi(argv[5]) != 0;
    }

    fd = open_can_socket(if_name);
    if (fd < 0)
    {
        return 1;
    }

    Chassis_CalculateWheelRpm(vy, vx, vw, wheel_rpm);
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        int16_t rpm_x10 = Chassis_EncodeRpmX10(wheel_rpm[i], fine);
        size_t len = ZdtX42s_BuildSpeedPayload(
            rpm_x10, ZDT_ACCELERATION, true, payload);
        uint32_t id = ZdtX42s_CommandId((uint8_t)(i + 1U), 0U);

        if ((len == 0U) || (send_frame(fd, id, payload, len) != 0))
        {
            close(fd);
            return 1;
        }
        print_frame(id, payload, len);
    }

    {
        size_t len = ZdtX42s_BuildSyncPayload(payload);
        uint32_t id = ZdtX42s_CommandId(0U, 0U);

        if ((len == 0U) || (send_frame(fd, id, payload, len) != 0))
        {
            close(fd);
            return 1;
        }
        print_frame(id, payload, len);
    }

    close(fd);
    return 0;
}
