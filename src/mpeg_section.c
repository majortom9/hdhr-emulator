#include "mpeg_section.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/dvb/dmx.h>

int mpeg_section_filter_open(int adapter, int demux_num, uint16_t pid,
                              uint8_t table_id, uint32_t timeout_ms)
{
    char path[64];
    snprintf(path, sizeof(path), "/dev/dvb/adapter%d/demux%d", adapter, demux_num);

    int fd = open(path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "mpeg_section: open %s failed: %s\n", path, strerror(errno));
        return -1;
    }

    struct dmx_sct_filter_params params;
    memset(&params, 0, sizeof(params));
    params.pid = pid;
    params.filter.filter[0] = table_id;
    params.filter.mask[0] = 0xFF; /* match table_id exactly; ignore the rest */
    params.timeout = timeout_ms;
    params.flags = DMX_CHECK_CRC | DMX_IMMEDIATE_START;

    if (ioctl(fd, DMX_SET_FILTER, &params) < 0) {
        fprintf(stderr, "mpeg_section: DMX_SET_FILTER (pid=0x%04X table_id=0x%02X) "
                         "failed: %s\n", pid, table_id, strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

ssize_t mpeg_section_read(int fd, uint8_t *buf, size_t bufsize, int timeout_ms)
{
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr == 0) return 0;   /* timeout */
    if (pr < 0) return -1;   /* poll error */

    ssize_t n = read(fd, buf, bufsize);
    if (n < 0) {
        if (errno == EOVERFLOW) return 0; /* section too large for our buffer; treat as miss */
        return -1;
    }
    return n;
}

void mpeg_section_filter_close(int fd)
{
    if (fd >= 0) {
        ioctl(fd, DMX_STOP);
        close(fd);
    }
}
