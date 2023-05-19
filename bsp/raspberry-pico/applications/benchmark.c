/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2021-01-28     flybreak       first version
 */
#include <rtthread.h>
#include <rtdevice.h>

static int num_thread = 2;
static int max_prime = 100000;
static int measure_time = 5;
int *cpu_bench_result;

/* Global value to record startup time */
rt_tick_t g_start;

int isPrime(int n)
{
#if 0
    int i;
    for (i = 2; i * i <= n; ++i)
    {
        if (n % i == 0)
            return 0;
    }
    return 1;
#endif
    int i, j = 1;
    for (i=0; i<10; i++) {
        j *= 5;
    }
    return 1;
}

int calculate_prime(int value, int max_time_in_sec)
{
    int i;
    rt_bool_t quit = RT_FALSE;
    rt_tick_t start, end;
    int result = 0;

    if (value == 0) {
    	value = 1000;
    }
    rt_enter_critical();
    if (g_start == 0) {
    	g_start = rt_tick_get();
    }
    rt_exit_critical();
    start = g_start;
    while (quit == RT_FALSE) {
        for (i = 0; i < value; i++){
            isPrime(i);
            end = rt_tick_get();
            if ((end-start)*1000/RT_TICK_PER_SECOND > max_time_in_sec) {
            	quit = RT_TRUE;
		result += i;
		goto quit;
            }
        }
	result += value;
    }
quit:

    return result;
}

void cpu_benchmark_entry(void* parameter)
{
    int thread = (int)parameter;
    cpu_bench_result[thread] = calculate_prime(max_prime, measure_time*1000);

    // rt_kprintf("Thread%d: %d\n", thread, cpu_bench_result[thread]);
}


#ifdef RT_USING_FINSH
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <finsh.h>
#include <msh_parse.h>

static int cpu_benchmark(int argc, char **argv)
{
    int i;
    rt_thread_t *pthread;
    char name[8];
    int total_result = 0;
    rt_kprintf("CPU benchmark\n");

    /* Reset value */
    g_start = 0;

    if (argc >= 2) {
        num_thread = atoi(argv[1]);
    }
    if (argc >= 3) {
        measure_time = atoi(argv[2]);
    }

    pthread = (rt_thread_t *)rt_malloc(sizeof(rt_thread_t)*num_thread);
    if (pthread == RT_NULL) {
        rt_kprintf("%s malloc mem fail\n", __func__);
    	goto exit;
    }
    cpu_bench_result = (int *)rt_malloc(sizeof(int)*num_thread);
    if (cpu_bench_result == RT_NULL) {
        rt_kprintf("%s malloc mem fail\n", __func__);
    	goto exit;
    }
    for (i=0; i<num_thread; i++) {
        rt_snprintf(name, sizeof(name), "%s[%d]", "c-b", i);
    	pthread[i] = rt_thread_create(name, cpu_benchmark_entry, (void *)i, 512, 21, 100/num_thread);
	if (pthread[i] != RT_NULL) rt_thread_startup(pthread[i]);
    }

    /* Wait measure_time + 1 for thread quit */
    rt_thread_mdelay((measure_time+1)*1000);
    rt_kprintf("CPU Benchmark Result:\n");
    for(i=0; i<num_thread; i++) {
    	rt_kprintf("\tThread%d %d\n", i, cpu_bench_result[i]);
    	total_result += cpu_bench_result[i];
    }
    rt_kprintf("Total Result: %d\n", total_result);

exit:
    if (pthread != RT_NULL) {
    	rt_free(pthread);
	pthread = NULL;
    }
    if (cpu_bench_result != RT_NULL) {
    	rt_free(cpu_bench_result);
	cpu_bench_result = NULL;
    }

    return 0;
}

MSH_CMD_EXPORT_ALIAS(cpu_benchmark, cpu_benchmark, cpu_benchmark [thread num] [time in sec]);
#endif
