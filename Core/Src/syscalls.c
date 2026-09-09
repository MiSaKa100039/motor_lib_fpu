/*
 * 文件说明：newlib 最小系统调用适配层。
 * 主要提供堆边界检查和无操作的 POSIX 桩函数，满足裸机链接需求。
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

extern uint8_t _end;
extern uint8_t _estack;
extern uint8_t _Min_Stack_Size;

int _close(int file)
{
    (void)file;
    errno = ENOSYS;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    (void)file;
    status->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int offset, int origin)
{
    (void)file;
    (void)offset;
    (void)origin;
    errno = ENOSYS;
    return -1;
}

int _read(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}

int _write(int file, const char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}

void *_sbrk(ptrdiff_t increment)
{
    static uint8_t *heap_end;
    uint8_t *previous_heap_end;
    uint8_t *stack_limit;

    if (heap_end == NULL)
    {
        heap_end = &_end;
    }

    stack_limit = &_estack - (uintptr_t)&_Min_Stack_Size;
    previous_heap_end = heap_end;

    if ((increment > 0) && ((size_t)(stack_limit - heap_end) < (size_t)increment))
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    if ((increment < 0) && ((size_t)(heap_end - &_end) < (size_t)(-increment)))
    {
        errno = EINVAL;
        return (void *)-1;
    }

    heap_end += increment;
    return previous_heap_end;
}

int _getpid(void)
{
    return 1;
}

int _kill(int process, int signal)
{
    (void)process;
    (void)signal;
    errno = EINVAL;
    return -1;
}

__attribute__((noreturn)) void _exit(int status)
{
    (void)status;

    for (;;)
    {
    }
}
