/*
 * Stress userfaultfd syscall.
 *
 *  Copyright (C) 2015  Red Hat, Inc.
 *
 *  This work is licensed under the terms of the GNU GPL, version 2. See
 *  the COPYING file in the top-level directory.
 *
 * This test allocates two virtual areas and bounces the physical
 * memory across the two virtual areas (from area_src to area_dst)
 * using userfaultfd.
 *
 * There are three threads running per CPU:
 *
 * 1) one per-CPU thread takes a per-page pthread_mutex in a random
 *    page of the area_dst (while the physical page may still be in
 *    area_src), and increments a per-page counter in the same page,
 *    and checks its value against a verification region.
 *
 * 2) another per-CPU thread handles the userfaults generated by
 *    thread 1 above. userfaultfd blocking reads or poll() modes are
 *    exercised interleaved.
 *
 * 3) one last per-CPU thread transfers the memory in the background
 *    at maximum bandwidth (if not already transferred by thread
 *    2). Each cpu thread takes cares of transferring a portion of the
 *    area.
 *
 * When all threads of type 3 completed the transfer, one bounce is
 * complete. area_src and area_dst are then swapped. All threads are
 * respawned and so the bounce is immediately restarted in the
 * opposite direction.
 *
 * per-CPU threads 1 by triggering userfaults inside
 * pthread_mutex_lock will also verify the atomicity of the memory
 * transfer (UFFDIO_COPY).
 *
 * The program takes two parameters: the amounts of physical memory in
 * megabytes (MiB) of the area and the number of bounces to execute.
 *
 * # 100MiB 99999 bounces
 * ./userfaultfd 100 99999
 *
 * # 1GiB 99 bounces
 * ./userfaultfd 1000 99
 *
 * # 10MiB-~6GiB 999 bounces, continue forever unless an error triggers
 * while ./userfaultfd $[RANDOM % 6000 + 10] 999; do true; done
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <poll.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include "../../../../include/uapi/linux/userfaultfd.h"

#ifdef __x86_64__
#define __NR_userfaultfd 323
#elif defined(__i386__)
#define __NR_userfaultfd 374
#elif defined(__powewrpc__)
#define __NR_userfaultfd 364
#else
#error "missing __NR_userfaultfd definition"
#endif

static unsigned long nr_cpus, nr_pages, nr_pages_per_cpu, page_size;

#define BOUNCE_RANDOM		(1<<0)
#define BOUNCE_RACINGFAULTS	(1<<1)
#define BOUNCE_VERIFY		(1<<2)
#define BOUNCE_POLL		(1<<3)
static int bounces;

static unsigned long long *count_verify;
static int uffd, finished, *pipefd;
static char *area_src, *area_dst;
static char *zeropage;
pthread_attr_t attr;

/* pthread_mutex_t starts at page offset 0 */
#define area_mutex(___area, ___nr)					\
	((pthread_mutex_t *) ((___area) + (___nr)*page_size))
/*
 * count is placed in the page after pthread_mutex_t naturally aligned
 * to avoid non alignment faults on non-x86 archs.
 */
#define area_count(___area, ___nr)					\
	((volatile unsigned long long *) ((unsigned long)		\
				 ((___area) + (___nr)*page_size +	\
				  sizeof(pthread_mutex_t) +		\
				  sizeof(unsigned long long) - 1) &	\
				 ~(unsigned long)(sizeof(unsigned long long) \
						  -  1)))

static int my_bcmp(char *str1, char *str2, size_t n)
{
	unsigned long i;
	for (i = 0; i < n; i++)
		if (str1[i] != str2[i])
			return 1;
	return 0;
}

static void *locking_thread(void *arg)
{
	unsigned long cpu = (unsigned long) arg;
	struct random_data rand;
	unsigned long page_nr = *(&(page_nr)); /* uninitialized warning */
	int32_t rand_nr;
	unsigned long long count;
	char randstate[64];
	unsigned int seed;
	time_t start;

	if (bounces & BOUNCE_RANDOM) {
		seed = (unsigned int) time(NULL) - bounces;
		if (!(bounces & BOUNCE_RACINGFAULTS))
			seed += cpu;
		bzero(&rand, sizeof(rand));
		bzero(&randstate, sizeof(randstate));
		if (initstate_r(seed, randstate, sizeof(randstate), &rand))
			fprintf(stderr, "srandom_r error\n"), exit(1);
	} else {
		page_nr = -bounces;
		if (!(bounces & BOUNCE_RACINGFAULTS))
			page_nr += cpu * nr_pages_per_cpu;
	}

	while (!finished) {
		if (bounces & BOUNCE_RANDOM) {
			if (random_r(&rand, &rand_nr))
				fprintf(stderr, "random_r 1 error\n"), exit(1);
			page_nr = rand_nr;
			if (sizeof(page_nr) > sizeof(rand_nr)) {
				if (random_r(&rand, &rand_nr))
					fprintf(stderr, "random_r 2 error\n"), exit(1);
				page_nr |= (((unsigned long) rand_nr) << 16) <<
					   16;
			}
		} else
			page_nr += 1;
		page_nr %= nr_pages;

		start = time(NULL);
		if (bounces & BOUNCE_VERIFY) {
			count = *area_count(area_dst, page_nr);
			if (!count)
				fprintf(stderr,
					"page_nr %lu wrong count %Lu %Lu\n",
					page_nr, count,
					count_verify[page_nr]), exit(1);


			/*
			 * We can't use bcmp (or memcmp) because that
			 * returns 0 erroneously if the memory is
			 * changing under it (even if the end of the
			 * page is never changing and always
			 * different).
			 */
#if 1
			if (!my_bcmp(area_dst + page_nr * page_size, zeropage,
				     page_size))
				fprintf(stderr,
					"my_bcmp page_nr %lu wrong count %Lu %Lu\n",
					page_nr, count,
					count_verify[page_nr]), exit(1);
#else
			unsigned long loops;

			loops = 0;
			/* uncomment the below line to test with mutex */
			/* pthread_mutex_lock(area_mutex(area_dst, page_nr)); */
			while (!bcmp(area_dst + page_nr * page_size, zeropage,
				     page_size)) {
				loops += 1;
				if (loops > 10)
					break;
			}
			/* uncomment below line to test with mutex */
			/* pthread_mutex_unlock(area_mutex(area_dst, page_nr)); */
			if (loops) {
				fprintf(stderr,
					"page_nr %lu all zero thread %lu %p %lu\n",
					page_nr, cpu, area_dst + page_nr * page_size,
					loops);
				if (loops > 10)
					exit(1);
			}
#endif
		}

		pthread_mutex_lock(area_mutex(area_dst, page_nr));
		count = *area_count(area_dst, page_nr);
		if (count != count_verify[page_nr]) {
			fprintf(stderr,
				"page_nr %lu memory corruption %Lu %Lu\n",
				page_nr, count,
				count_verify[page_nr]), exit(1);
		}
		count++;
		*area_count(area_dst, page_nr) = count_verify[page_nr] = count;
		pthread_mutex_unlock(area_mutex(area_dst, page_nr));

		if (time(NULL) - start > 1)
			fprintf(stderr,
				"userfault too slow %ld "
				"possible false positive with overcommit\n",
				time(NULL) - start);
	}

	return NULL;
}

static int copy_page(unsigned long offset)
{
	struct uffdio_copy uffdio_copy;

	if (offset >= nr_pages * page_size)
		fprintf(stderr, "unexpected offset %lu\n",
			offset), exit(1);
	uffdio_copy.dst = (unsigned long) area_dst + offset;
	uffdio_copy.src = (unsigned long) area_src + offset;
	uffdio_copy.len = page_size;
	uffdio_copy.mode = 0;
	uffdio_copy.copy = 0;
	if (ioctl(uffd, UFFDIO_COPY, &uffdio_copy)) {
		/* real retval in ufdio_copy.copy */
		if (uffdio_copy.copy != -EEXIST)
			fprintf(stderr, "UFFDIO_COPY error %Ld\n",
				uffdio_copy.copy), exit(1);
	} else if (uffdio_copy.copy != page_size) {
		fprintf(stderr, "UFFDIO_COPY unexpected copy %Ld\n",
			uffdio_copy.copy), exit(1);
	} else
		return 1;
	return 0;
}

static void *uffd_poll_thread(void *arg)
{
	unsigned long cpu = (unsigned long) arg;
	struct pollfd pollfd[2];
	struct uffd_msg msg;
	int ret;
	unsigned long offset;
	char tmp_chr;
	unsigned long userfaults = 0;

	pollfd[0].fd = uffd;
	pollfd[0].events = POLLIN;
	pollfd[1].fd = pipefd[cpu*2];
	pollfd[1].events = POLLIN;

	for (;;) {
		ret = poll(pollfd, 2, -1);
		if (!ret)
			fprintf(stderr, "poll error %d\n", ret), exit(1);
		if (ret < 0)
			perror("poll"), exit(1);
		if (pollfd[1].revents & POLLIN) {
			if (read(pollfd[1].fd, &tmp_chr, 1) != 1)
				fprintf(stderr, "read pipefd error\n"),
					exit(1);
			break;
		}
		if (!(pollfd[0].revents & POLLIN))
			fprintf(stderr, "pollfd[0].revents %d\n",
				pollfd[0].revents), exit(1);
		ret = read(uffd, &msg, sizeof(msg));
		if (ret < 0) {
			if (errno == EAGAIN)
				continue;
			perror("nonblocking read error"), exit(1);
		}
		if (msg.event != UFFD_EVENT_PAGEFAULT)
			fprintf(stderr, "unexpected msg event %u\n",
				msg.event), exit(1);
		if (msg.arg.pagefault.flags & UFFD_PAGEFAULT_FLAG_WRITE)
			fprintf(stderr, "unexpected write fault\n"), exit(1);
		offset = (char *)(unsigned long)msg.arg.pagefault.address -
			 area_dst;
		offset &= ~(page_size-1);
		if (copy_page(offset))
			userfaults++;
	}
	return (void *)userfaults;
}

pthread_mutex_t uffd_read_mutex = PTHREAD_MUTEX_INITIALIZER;

static void *uffd_read_thread(void *arg)
{
	unsigned long *this_cpu_userfaults;
	struct uffd_msg msg;
	unsigned long offset;
	int ret;

	this_cpu_userfaults = (unsigned long *) arg;
	*this_cpu_userfaults = 0;

	pthread_mutex_unlock(&uffd_read_mutex);
	/* from here cancellation is ok */

	for (;;) {
		ret = read(uffd, &msg, sizeof(msg));
		if (ret != sizeof(msg)) {
			if (ret < 0)
				perror("blocking read error"), exit(1);
			else
				fprintf(stderr, "short read\n"), exit(1);
		}
		if (msg.event != UFFD_EVENT_PAGEFAULT)
			fprintf(stderr, "unexpected msg event %u\n",
				msg.event), exit(1);
		if (bounces & BOUNCE_VERIFY &&
		    msg.arg.pagefault.flags & UFFD_PAGEFAULT_FLAG_WRITE)
			fprintf(stderr, "unexpected write fault\n"), exit(1);
		offset = (char *)(unsigned long)msg.arg.pagefault.address -
			 area_dst;
		offset &= ~(page_size-1);
		if (copy_page(offset))
			(*this_cpu_userfaults)++;
	}
	return (void *)NULL;
}

static void *background_thread(void *arg)
{
	unsigned long cpu = (unsigned long) arg;
	unsigned long page_nr;

	for (page_nr = cpu * nr_pages_per_cpu;
	     page_nr < (cpu+1) * nr_pages_per_cpu;
	     page_nr++)
		copy_page(page_nr * page_size);

	return NULL;
}

static int stress(unsigned long *userfaults)
{
	unsigned long cpu;
	pthread_t locking_threads[nr_cpus];
	pthread_t uffd_threads[nr_cpus];
	pthread_t background_threads[nr_cpus];
	void **_userfaults = (void **) userfaults;

	finished = 0;
	for (cpu = 0; cpu < nr_cpus; cpu++) {
		if (pthread_create(&locking_threads[cpu], &attr,
				   locking_thread, (void *)cpu))
			return 1;
		if (bounces & BOUNCE_POLL) {
			if (pthread_create(&uffd_threads[cpu], &attr,
					   uffd_poll_thread, (void *)cpu))
				return 1;
		} else {
			if (pthread_create(&uffd_threads[cpu], &attr,
					   uffd_read_thread,
					   &_userfaults[cpu]))
				return 1;
			pthread_mutex_lock(&uffd_read_mutex);
		}
		if (pthread_create(&background_threads[cpu], &attr,
				   background_thread, (void *)cpu))
			return 1;
	}
	for (cpu = 0; cpu < nr_cpus; cpu++)
		if (pthread_join(background_threads[cpu], NULL))
			return 1;

	/*
	 * Be strict and immediately zap area_src, the whole area has
	 * been transferred already by the background treads. The
	 * area_src could then be faulted in in a racy way by still
	 * running uffdio_threads reading zeropages after we zapped
	 * area_src (but they're guaranteed to get -EEXIST from
	 * UFFDIO_COPY without writing zero pages into area_dst
	 * because the background threads already completed).
	 */
	if (madvise(area_src, nr_pages * page_size, MADV_DONTNEED)) {
		perror("madvise");
		return 1;
	}

	for (cpu = 0; cpu < nr_cpus; cpu++) {
		char c;
		if (bounces & BOUNCE_POLL) {
			if (write(pipefd[cpu*2+1], &c, 1) != 1) {
				fprintf(stderr, "pipefd write error\n");
				return 1;
			}
			if (pthread_join(uffd_threads[cpu], &_userfaults[cpu]))
				return 1;
		} else {
			if (pthread_cancel(uffd_threads[cpu]))
				return 1;
			if (pthread_join(uffd_threads[cpu], NULL))
				return 1;
		}
	}

	finished = 1;
	for (cpu = 0; cpu < nr_cpus; cpu++)
		if (pthread_join(locking_threads[cpu], NULL))
			return 1;

	return 0;
}

static int userfaultfd_stress(void)
{
	void *area;
	char *tmp_area;
	unsigned long nr;
	struct uffdio_register uffdio_register;
	struct uffdio_api uffdio_api;
	unsigned long cpu;
	int uffd_flags;
	unsigned long userfaults[nr_cpus];

	if (posix_memalign(&area, page_size, nr_pages * page_size)) {
		fprintf(stderr, "out of memory\n");
		return 1;
	}
	area_src = area;
	if (posix_memalign(&area, page_size, nr_pages * page_size)) {
		fprintf(stderr, "out of memory\n");
		return 1;
	}
	area_dst = area;

	uffd = syscall(__NR_userfaultfd, O_CLOEXEC | O_NONBLOCK);
	if (uffd < 0) {
		fprintf(stderr,
			"userfaultfd syscall not available in this kernel\n");
		return 1;
	}
	uffd_flags = fcntl(uffd, F_GETFD, NULL);

	uffdio_api.api = UFFD_API;
	uffdio_api.features = 0;
	if (ioctl(uffd, UFFDIO_API, &uffdio_api)) {
		fprintf(stderr, "UFFDIO_API\n");
		return 1;
	}
	if (uffdio_api.api != UFFD_API) {
		fprintf(stderr, "UFFDIO_API error %Lu\n", uffdio_api.api);
		return 1;
	}

	count_verify = malloc(nr_pages * sizeof(unsigned long long));
	if (!count_verify) {
		perror("count_verify");
		return 1;
	}

	for (nr = 0; nr < nr_pages; nr++) {
		*area_mutex(area_src, nr) = (pthread_mutex_t)
			PTHREAD_MUTEX_INITIALIZER;
		count_verify[nr] = *area_count(area_src, nr) = 1;
	}

	pipefd = malloc(sizeof(int) * nr_cpus * 2);
	if (!pipefd) {
		perror("pipefd");
		return 1;
	}
	for (cpu = 0; cpu < nr_cpus; cpu++) {
		if (pipe2(&pipefd[cpu*2], O_CLOEXEC | O_NONBLOCK)) {
			perror("pipe");
			return 1;
		}
	}

	if (posix_memalign(&area, page_size, page_size)) {
		fprintf(stderr, "out of memory\n");
		return 1;
	}
	zeropage = area;
	bzero(zeropage, page_size);

	pthread_mutex_lock(&uffd_read_mutex);

	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 16*1024*1024);

	while (bounces--) {
		unsigned long expected_ioctls;

		printf("bounces: %d, mode:", bounces);
		if (bounces & BOUNCE_RANDOM)
			printf(" rnd");
		if (bounces & BOUNCE_RACINGFAULTS)
			printf(" racing");
		if (bounces & BOUNCE_VERIFY)
			printf(" ver");
		if (bounces & BOUNCE_POLL)
			printf(" poll");
		printf(", ");
		fflush(stdout);

		if (bounces & BOUNCE_POLL)
			fcntl(uffd, F_SETFL, uffd_flags | O_NONBLOCK);
		else
			fcntl(uffd, F_SETFL, uffd_flags & ~O_NONBLOCK);

		/* register */
		uffdio_register.range.start = (unsigned long) area_dst;
		uffdio_register.range.len = nr_pages * page_size;
		uffdio_register.mode = UFFDIO_REGISTER_MODE_MISSING;
		if (ioctl(uffd, UFFDIO_REGISTER, &uffdio_register)) {
			fprintf(stderr, "register failure\n");
			return 1;
		}
		expected_ioctls = (1 << _UFFDIO_WAKE) |
				  (1 << _UFFDIO_COPY) |
				  (1 << _UFFDIO_ZEROPAGE);
		if ((uffdio_register.ioctls & expected_ioctls) !=
		    expected_ioctls) {
			fprintf(stderr,
				"unexpected missing ioctl for anon memory\n");
			return 1;
		}

		/*
		 * The madvise done previously isn't enough: some
		 * uffd_thread could have read userfaults (one of
		 * those already resolved by the background thread)
		 * and it may be in the process of calling
		 * UFFDIO_COPY. UFFDIO_COPY will read the zapped
		 * area_src and it would map a zero page in it (of
		 * course such a UFFDIO_COPY is perfectly safe as it'd
		 * return -EEXIST). The problem comes at the next
		 * bounce though: that racing UFFDIO_COPY would
		 * generate zeropages in the area_src, so invalidating
		 * the previous MADV_DONTNEED. Without this additional
		 * MADV_DONTNEED those zeropages leftovers in the
		 * area_src would lead to -EEXIST failure during the
		 * next bounce, effectively leaving a zeropage in the
		 * area_dst.
		 *
		 * Try to comment this out madvise to see the memory
		 * corruption being caught pretty quick.
		 *
		 * khugepaged is also inhibited to collapse THP after
		 * MADV_DONTNEED only after the UFFDIO_REGISTER, so it's
		 * required to MADV_DONTNEED here.
		 */
		if (madvise(area_dst, nr_pages * page_size, MADV_DONTNEED)) {
			perror("madvise 2");
			return 1;
		}

		/* bounce pass */
		if (stress(userfaults))
			return 1;

		/* unregister */
		if (ioctl(uffd, UFFDIO_UNREGISTER, &uffdio_register.range)) {
			fprintf(stderr, "register failure\n");
			return 1;
		}

		/* verification */
		if (bounces & BOUNCE_VERIFY) {
			for (nr = 0; nr < nr_pages; nr++) {
				if (my_bcmp(area_dst,
					    area_dst + nr * page_size,
					    sizeof(pthread_mutex_t))) {
					fprintf(stderr,
						"error mutex 2 %lu\n",
						nr);
					bounces = 0;
				}
				if (*area_count(area_dst, nr) != count_verify[nr]) {
					fprintf(stderr,
						"error area_count %Lu %Lu %lu\n",
						*area_count(area_src, nr),
						count_verify[nr],
						nr);
					bounces = 0;
				}
			}
		}

		/* prepare next bounce */
		tmp_area = area_src;
		area_src = area_dst;
		area_dst = tmp_area;

		printf("userfaults:");
		for (cpu = 0; cpu < nr_cpus; cpu++)
			printf(" %lu", userfaults[cpu]);
		printf("\n");
	}

	return 0;
}

int main(int argc, char **argv)
{
	if (argc < 3)
		fprintf(stderr, "Usage: <MiB> <bounces>\n"), exit(1);
	nr_cpus = sysconf(_SC_NPROCESSORS_ONLN);
	page_size = sysconf(_SC_PAGE_SIZE);
	if ((unsigned long) area_count(NULL, 0) + sizeof(unsigned long long) >
	    page_size)
		fprintf(stderr, "Impossible to run this test\n"), exit(2);
	nr_pages_per_cpu = atol(argv[1]) * 1024*1024 / page_size /
		nr_cpus;
	if (!nr_pages_per_cpu) {
		fprintf(stderr, "invalid MiB\n");
		fprintf(stderr, "Usage: <MiB> <bounces>\n"), exit(1);
	}
	bounces = atoi(argv[2]);
	if (bounces <= 0) {
		fprintf(stderr, "invalid bounces\n");
		fprintf(stderr, "Usage: <MiB> <bounces>\n"), exit(1);
	}
	nr_pages = nr_pages_per_cpu * nr_cpus;
	printf("nr_pages: %lu, nr_pages_per_cpu: %lu\n",
	       nr_pages, nr_pages_per_cpu);
	return userfaultfd_stress();
}
