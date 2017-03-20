
#define SYSFS_BUF_SIZE		(4<<10)

int sysfs_write(char *full_path, char *buf, int size)
{
	char *w_buf = buf;
	int fd;
	int offset = 0;
	int w_size = 0;
	int ret = 0;

	fd = open(full_path, O_RDWR);
	if (fd < 0) {
		printf("can't open %s\n", w->full_path);
		ret = -EINVAL;
		goto out;
	}

	while (size) {
		w_size = size;
		if (w_size > SYSFS_BUF_SIZE) {
			w_size = SYSFS_BUF_SIZE;
		}

		ret = write(fd, w_buf, w_size);
		if (ret < 0) {
			printf("can't write attr(%s), %d(%s), %Xh(%Xh)\n",
				full_path, ret, strerror(ret),
				offset, w_size);
			break;
		}

		offset += w_size;
		w_buf += w_size;
		size -= w_size;
	}

	close(fd);

out:
	return ret;
}


