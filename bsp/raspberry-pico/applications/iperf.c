#include "rtthread.h"

#include <lwip/sockets.h>

#include "iperf.h"

#include "lwip/opt.h"

#include "lwip/sys.h"
#include "lwip/api.h"

#define IPERF_PORT          5001
#define IPERF_BUFSZ         (4 * 1024)
#define IPERF_TIMEOUT 		500

volatile static rt_bool_t iperf_need_quit = RT_FALSE;

void iperf_server(void *thread_param)
{
	struct netconn *conn, *newconn;
	err_t err;

	iperf_need_quit = RT_FALSE;

	conn = netconn_new(NETCONN_TCP);
	netconn_set_recvtimeout(conn, IPERF_TIMEOUT);
	netconn_bind(conn, IP_ADDR_ANY, 5001);

	LWIP_ERROR("tcpecho: invalid conn", (conn != NULL), return;);

	/* Tell connection to go into listening mode. */
	netconn_listen(conn);

	while (1) {
		if (iperf_need_quit == RT_TRUE) {
			break;
		}
		/* Grab new connection. */
		err = netconn_accept(conn, &newconn);
		/* Process the new connection. */
		if (err == ERR_OK) {
			struct netbuf *buf;
			void *data;
			u16_t len;

			rt_kprintf("accepted new connection %p\n", newconn);
			netconn_set_recvtimeout(newconn, IPERF_TIMEOUT);
			while (1) {
				err = netconn_recv(newconn, &buf);
				if (err == ERR_OK) {
					do {
						netbuf_data(buf, &data, &len);
#if 0
						err =
						    netconn_write(newconn, data,
								  len,
								  NETCONN_COPY);
						if (err != ERR_OK) {
							break;
						}
#endif
					}
					while (netbuf_next(buf) >= 0);
					netbuf_delete(buf);
				} else if (err == ERR_TIMEOUT) {
					continue;
				} else {
					rt_kprintf("netconn_recv: errno: %d\n",
						   err);
					break;
				}
				if (iperf_need_quit == RT_TRUE) {
					break;
				}
			}
			rt_kprintf("Got EOF, looping\n");
			netconn_delete(newconn);
		}
	}
	netconn_delete(conn);
	rt_kprintf("%s quit\n", __func__);
}

static void iperf_test(int argc, char **argv)
{
	rt_thread_t tid;
	rt_uint32_t retry = 5;

	while (--retry > 0) {
		tid = rt_thread_find(IPERF_THREAD_NAME);
		if (tid != (rt_thread_t) RT_NULL) {
			// Force thread quit
			iperf_need_quit = RT_TRUE;
		} else {
			break;
		}
		rt_thread_delay(IPERF_TIMEOUT + 10);
	}
	if (retry == 0) {
		rt_kprintf("%s: %s is already running, quit fail\n", __func__,
			   IPERF_THREAD_NAME);
		return;
	}

	tid = rt_thread_create(IPERF_THREAD_NAME, iperf_server, RT_NULL,
			       RT_IPERF_THREAD_STACK_SIZE,
			       RT_IPERF_THREAD_PRIORITY, 20);
	RT_ASSERT(tid != RT_NULL);
	rt_thread_startup(tid);
}

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT_ALIAS(iperf_test, iperf, Start iperf server / client);
#endif
