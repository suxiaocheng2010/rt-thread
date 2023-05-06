#ifndef LWIP_IPERF_H
#define LWIP_IPERF_H

#define IPERF_THREAD_NAME            "iperf"

#define RT_IPERF_THREAD_STACK_SIZE 	4096
#define RT_IPERF_THREAD_PRIORITY 	15

void iperf_server(void *thread_param);
void iperf_server_init(void);

#endif
