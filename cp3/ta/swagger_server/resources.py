import psutil
import resource

def bytes_to_gigabytes(x):
    return x / 1024 / 1024 / 1024


def report_system_resources(logger):
    cores_physical = psutil.cpu_count(logical=False)
    cores_logical = psutil.cpu_count(logical=True)
    cpu_freq = psutil.cpu_freq().max
    vmem_total = bytes_to_gigabytes(psutil.virtual_memory().total)
    swap_total = bytes_to_gigabytes(psutil.swap_memory().total)
    swap_free = bytes_to_gigabytes(psutil.swap_memory().free)
    disk_info = psutil.disk_usage('/')
    disk_size = bytes_to_gigabytes(disk_info.total)
    disk_free = bytes_to_gigabytes(disk_info.free)

    resource_s = '\n'.join([
        '* CPU cores: {} physical, {} logical'.format(cores_physical, cores_logical),  # noqa: pycodestyle
        '* CPU frequency: {:.2f} GHz'.format(cpu_freq / 1000),
        '* virtual memory: {:.2f} GB'.format(vmem_total),
        '* swap memory: {:.2f} GB ({:.2f} GB free)'.format(swap_total, swap_free),  # noqa: pycodestyle
        '* disk space: {:.2f} GB ({:.2f} GB free)'.format(disk_size, disk_free)  # noqa: pycodestyle
    ])
    logger.info("system resources:\n%s", resource_s)


def report_resource_limits(logger):
    resources = [
        ('CPU time (seconds)', resource.RLIMIT_CPU),
        ('Heap size (bytes)', resource.RLIMIT_DATA),
        ('Num. process', resource.RLIMIT_NPROC),
        ('Num. files', resource.RLIMIT_NOFILE),
        ('Address space', resource.RLIMIT_AS),
        ('Locked address space', resource.RLIMIT_MEMLOCK)
    ]
    resource_limits = [
        (name, resource.getrlimit(res)) for (name, res) in resources
    ]
    resource_s = '\n'.join([
        '* {}: {}'.format(res, lim) for (res, lim) in resource_limits
    ])
    logger.info("resource limits:\n%s", resource_s)
