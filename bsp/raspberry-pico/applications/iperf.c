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

void iperf_client(void *thread_param)
{
	struct netconn *conn;

	int i;

	int ret;

	uint8_t *send_buf;

	uint64_t sentlen;

	u32_t tick1, tick2;
	ip4_addr_t ipaddr;

	iperf_need_quit = RT_FALSE;

	send_buf = (uint8_t *) rt_malloc(IPERF_BUFSZ);
	if (!send_buf)
		return;

	for (i = 0; i < IPERF_BUFSZ; i++)
		send_buf[i] = i & 0xff;

	while (1) {
		conn = netconn_new(NETCONN_TCP);
		if (iperf_need_quit == RT_TRUE) {
			break;
		}
		if (conn == NULL) {
			// rt_kprintf("create conn failed!\n");
			rt_thread_delay(10);
			continue;
		}

		IP4_ADDR(&ipaddr, 192, 168, 1, 1);

		ret = netconn_connect(conn, &ipaddr, 5001);
		if (ret == -1) {
			rt_kprintf("Connect failed!\n");
			netconn_close(conn);
			rt_thread_delay(10);
			continue;
		}

		rt_kprintf("Connect to iperf server successful!\n");

		tick1 = sys_jiffies();
		while (1) {
			tick2 = sys_jiffies();

			if (tick2 - tick1 >= RT_TICK_PER_SECOND) {
				float f;
				int integer, fract;
				f = (float)(sentlen * RT_TICK_PER_SECOND/ 125 /
					    (tick2 - tick1));
				f /= 1000.0f;
				integer = (int)f;
				fract = (f-integer)*1000;
				rt_kprintf("send speed = %d.%d Mbps!\n", integer, fract);

				tick1 = tick2;
				sentlen = 0;
			}
			ret = netconn_write(conn, send_buf, IPERF_BUFSZ, 0);
			if (ret == ERR_OK) {
				sentlen += IPERF_BUFSZ;
			}
			if (iperf_need_quit == RT_TRUE) {
				break;
			}
		}
	}
	if (conn != NULL) {
		netconn_close(conn);
		netconn_delete(conn);
		conn = NULL;
	}

	if (send_buf) {
		rt_free(send_buf);
	}
}

static void iperf_test(int argc, char **argv)
{
	rt_thread_t tid;
	rt_uint32_t retry = 5;
	rt_bool_t client = RT_TRUE;

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

	if (argc >= 2) {
		if(rt_strcmp(argv[1], "-s") == 0){
			client = RT_FALSE;
		} else {
			rt_kprintf("unknown option: %s\n", argv[1]);
		}
	}
	tid = rt_thread_create(IPERF_THREAD_NAME, client?iperf_client:iperf_server, RT_NULL,
			       RT_IPERF_THREAD_STACK_SIZE,
			       RT_IPERF_THREAD_PRIORITY, 20);
	RT_ASSERT(tid != RT_NULL);
	rt_thread_startup(tid);
}

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT_ALIAS(iperf_test, iperf, Start iperf server / client);
#endif
