/**
 * @file freestanding_errno.c
 * @brief 裸机 errno 兼容。
 */

static int freestanding_libm_errno_storage = 0;

int *__errno(void)
{
    return &freestanding_libm_errno_storage;
}
