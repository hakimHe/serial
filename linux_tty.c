#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/serial.h>
#include <asm-generic/ioctls.h>
#include <sys/time.h>

#define MAX_BUF_SIZE 10240
static char* write_buf = NULL;
static char* read_buf = NULL;
int tty_rs485_mode = 0;

#define GPIO_RS485_PIN "17"
#define GPIO_RS485_SET_HIGH "1"
#define GPIO_RS485_SET_LOW 	"0"

int tty_get_baudrate(int fd)
{
	struct termios options;
	int baudrate, i;
	int baudrate_macro[] = {B460800, B115200, B38400, B19200, B9600, B4800};
	int baudrate_num[] =   { 460800,  115200,  38400,  19200,  9600,  4800};

	if(tcgetattr(fd, &options) != 0) {
		perror("tcgetattr fail");
		return -1;
	}

	baudrate = cfgetispeed(&options);

	for(i=0; i<sizeof(baudrate_macro)/sizeof(int); i++) {
		if(baudrate == baudrate_macro[i]) {
			return baudrate_num[i];
		}
	}

	return -1;
}

int tty_set_rs485_mode()
{
	int export_fd, direction_fd;

	if(!tty_rs485_mode) return 0;

	export_fd = open("/sys/class/gpio/export", O_WRONLY);
	if(-1 != export_fd) {
		if(-1 == write(export_fd, GPIO_RS485_PIN, sizeof(GPIO_RS485_PIN)))
		{
			printf("gpio %s write file operation error, maybe it is already export!\n", GPIO_RS485_PIN);
			close(export_fd);
		} else {
			close(export_fd);
			printf("tty_set_opt gpio %s export!\n", GPIO_RS485_PIN);
		}
	}

	direction_fd = open("/sys/class/gpio/gpio17/direction", O_WRONLY);
	if(-1 == direction_fd)
	{
		printf("open gpio direction file error\r\n");
		return -1;
	}
	if(-1 == write(direction_fd, "out", sizeof("out")))
	{
		printf("write operation direction error\n");
		close(direction_fd);
		return -1;
	}
	close(direction_fd);

	return 0;
}

void tcdrain_delay(int fd)
{
	int usec = 0, baudrate = 0;
	baudrate = tty_get_baudrate(fd);
	usec = (1 / (float)(baudrate / 10)) * 128 * 1000000 + 10000;
	usleep(usec);
}

int tty_set_gpio_value(char *set_value)
{
	int gpiovalue_fd;

	if(!tty_rs485_mode) return 0;

	gpiovalue_fd = open("/sys/class/gpio/gpio17/value", O_WRONLY);
	if(-1 == gpiovalue_fd)
	{
		printf("open value file error\r\n");
		return -1;
	}

	if(-1 == write(gpiovalue_fd, set_value, sizeof(set_value)))
	{
		printf("write value %s operation value error\n", set_value);
		close(gpiovalue_fd);
		return -1;
	}
	close(gpiovalue_fd);

	return 0;
}

static int alloc_buf(void)
{
	if(!write_buf) {
		write_buf = (char*)malloc(MAX_BUF_SIZE);
		if (!write_buf) {
			printf("buff malloc fail!\n");
			return -1;
		}
	}
	memset(write_buf, 0, MAX_BUF_SIZE);

	if(!read_buf) {
		read_buf = (char*)malloc(MAX_BUF_SIZE);
		if (!read_buf) {
			printf("buff malloc fail!\n");
			free(write_buf);
			return -1;
		}
	}
	memset(read_buf, 0, MAX_BUF_SIZE);

	return 0;
}

static void free_buf(void)
{
	free(write_buf);
	write_buf = NULL;
	free(read_buf);
	read_buf = NULL;
}

int file2buf(char *filename, char *buf, int buf_len)
{
	int flen;
	FILE *fp;

	fp = fopen(filename, "r");
	if (fp ==NULL) {
		printf("open %s fail!\n", filename);
		return -1;
	}

	fseek(fp,0L,SEEK_END);
	flen = ftell(fp);
	if(flen > buf_len) {
		printf("file2buf flen %d > buf_len %d\n", flen, buf_len);
		fclose(fp);
		return -1;
	}

	fseek(fp, 0L, SEEK_SET);
	fread(buf, flen, 1, fp);
	fclose(fp);

	return flen;
}

int buf2file(char *filename, char *buf, int buf_len)
{
	FILE *fp;

	if(!buf_len)
		return -1;

	fp = fopen(filename, "w+");
	if (fp ==NULL) {
		printf("open %s fail!\n", filename);
		return -1;
	}

	fwrite(buf, buf_len, 1, fp);
	fclose(fp);
	return 0;
}

int tty_open(char* dev_name)
{
	int fd;
	fd = open(dev_name, O_RDWR|O_NOCTTY);
	if (fd < 0) {
		printf("can`t open tty %s\n", dev_name);
		return -1;
	}

#if 0
	usleep(200000);
	if(tcflush(fd ,TCIOFLUSH) < 0) {
		printf("tty_open tcflush TCIOFLUSH fail!\n");
	}
	usleep(200000);
#endif
	if(fcntl(fd, F_SETFL, 0) < 0) {
		printf("fcntl failed!\n");
		close(fd);
		return -1;
	} else {
		printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
	}

	if(0 == isatty(STDIN_FILENO)) {
		printf("standard input is not a terminal device\n");
		close(fd);
		return -1;
	}

	printf("open %s success!\n", dev_name);
	return fd;
}

void tty_close(int fd)
{
	close(fd);
	return;
}

int tty_send(int fd, char* buf, int len)
{
    int ret;

	tty_set_gpio_value(GPIO_RS485_SET_HIGH);
	ret = write(fd,buf,len);
	tcdrain(fd);
	tcdrain_delay(fd);
	tty_set_gpio_value(GPIO_RS485_SET_LOW);

	if (len == ret ) {
		return ret;
	} else {
		printf("tty_send fail, TCOFLUSH\n");
		tcflush(fd,TCOFLUSH);
		usleep(100000);
		return -1;
	}
	return -1;
}

int tty_recv(int fd, char* buf, int len)
{
	int fs_sel, baudrate, act_len=0, count;
	fd_set fs_read;
	struct timeval time;
	float usec = 0;

	tty_set_gpio_value(GPIO_RS485_SET_LOW);

	baudrate = tty_get_baudrate(fd);
	usec = (1 / (float)(baudrate / 10)) * 128 * 1000000;
	time.tv_sec = (int)usec / 1000000;
	time.tv_usec = (int)usec % 1000000 + 10000;

	while(1) {
		FD_ZERO(&fs_read);
		FD_SET(fd,&fs_read);

		if(act_len == 0) {
			time.tv_sec = 5;
			time.tv_usec = 0;
		} else {
			time.tv_sec = (int)usec / 1000000;
			time.tv_usec = (int)usec % 1000000 + 10000;
		}

		fs_sel = select(fd+1, &fs_read, NULL, NULL, &time);
		if (fs_sel == -1) {
			printf("select error:%s.\n", strerror(errno));
			return act_len;
		}else if (fs_sel == 0) {
			printf("tty_recv select timeout\n");
			return act_len;
		} else {
			if (FD_ISSET(fd, &fs_read)) {
				count = read(fd, buf+act_len, len-act_len);
				act_len = (count > 0) ? (act_len+count) : act_len;
				printf("[%d] [%d]\n", count, act_len);
				if(act_len >= len) {
					return act_len;
				}
			}
		}
	}
}

int tty_set_opt(int fd, int rs485, int baudrate, int bits, int stop, int flow_ctl, char parity)
{
	struct termios options;
	struct serial_rs485 rs485conf;
	int baudrate_macro[] = {B460800, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
	int baudrate_num[] =   { 460800,  115200,  38400,  19200,  9600,  4800,  2400,  1200,  300};
	int i, export_fd;

	if(tcgetattr(fd,&options) != 0) {
		perror("tcgetattr fail");
		return -1;
	}

	options.c_cflag |= (CLOCAL|CREAD ); /*CREAD 开启串行数据接收，CLOCAL并打开本地连接模式*/
	options.c_cflag &=~CSIZE;/*设置数据位*/

	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_iflag &= ~(ICRNL | INLCR);
	options.c_oflag &= ~(OCRNL | ONLCR);

	//设置数据流控制
	switch(flow_ctl)
	{
	case 0 ://不使用流控制
		options.c_cflag &= ~CRTSCTS;
		break;
	case 1 ://使用硬件流控制
		options.c_cflag |= CRTSCTS;
		break;
	case 2 ://使用软件流控制
		options.c_cflag |= IXON | IXOFF | IXANY;
		break;
	}

	if(rs485) {
		tty_rs485_mode = 1;
		tty_set_rs485_mode();
	} else {
		tty_rs485_mode = 0;
	}

	switch(bits) {
		case 7:
			options.c_cflag |=CS7;
			break;
		case 8:
			options.c_cflag |=CS8;
			break;
		default:
			break;
	}

	switch(parity) {
		case 'o':  /*奇校验*/
		case 'O':
			options.c_cflag |= PARENB;/*开启奇偶校验*/
			options.c_iflag |= (INPCK | ISTRIP);/*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */
			options.c_cflag |= PARODD;/*启用奇校验(默认为偶校验)*/
			break;
		case 'e':/*偶校验*/
		case 'E':
			options.c_cflag |= PARENB; /*开启奇偶校验  */
			options.c_iflag |= ( INPCK | ISTRIP);/*打开输入奇偶校验并去除字符第八个比特*/
			options.c_cflag &= ~PARODD;/*启用偶校验*/
			break;
		case 'n': /*无奇偶校验*/
		case 'N':
			options.c_cflag &= ~PARENB;
			break;
		default:
			break;
	}

	for(i= 0;i < sizeof(baudrate_num) / sizeof(int);i++) {
		if	(baudrate == baudrate_num[i]) {
			cfsetispeed(&options, baudrate_macro[i]);
			cfsetospeed(&options, baudrate_macro[i]);
			break;
		}
	}

	if(stop == 1){/*设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB*/
		options.c_cflag &= ~CSTOPB;/*默认为一位停止位； */
	}
	else if(stop == 2) {
		options.c_cflag |= CSTOPB;/*CSTOPB表示送两位停止位*/
	}
	//只是串口传输数据，而不需要串口来处理，那么使用原始模式(Raw Mode)方式来通讯
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	options.c_oflag  &= ~OPOST;   /*Output*/

	/*设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时*/
	options.c_cc[VTIME] = 0;/*非规范模式读取时的超时时间；*/
	options.c_cc[VMIN]  = 0; /*非规范模式读取时的最小字符数*/

	if(tcflush(fd ,TCIFLUSH) < 0) {
		printf("tcflush TCIOFLUSH fail!\n");
	}
	//usleep(200000);

	/*激活配置使其生效*/
	if((tcsetattr(fd, TCSANOW, &options))!=0) {
		perror("com set error");
		return -1;
	}

	printf("set_termios success!\n");
    return 0;
}

static void usage(const char* proc)
{
	printf("Usage: %s [/dev/ttySn] [0:rs232; 1:rs485] [baudrate] [bits] [stop] [parity] [flow_control] [0:send and recv/1:recv and send/2:send only/3:recv only]\n", proc);
	printf("rs232 send and recv: %s /dev/ttyACM1 0 9600 8 1 N 0 0 255\n", proc);
	printf("rs232 recv and send: %s /dev/ttyACM1 0 9600 8 1 N 0 1 255\n", proc);
	printf("rs232 send:          %s /dev/ttyACM1 0 9600 8 1 N 0 2 255\n", proc);
	printf("rs232 recv:          %s /dev/ttyACM1 0 9600 8 1 N 0 3 255\n", proc);
	printf("rs485 send and recv: %s /dev/ttyACM0 1 9600 8 1 N 0 0 255\n", proc);
	printf("rs485 recv and send: %s /dev/ttyACM0 1 9600 8 1 N 0 1 255\n", proc);
	printf("rs485 send:          %s /dev/ttyACM0 1 9600 8 1 N 0 2 255\n", proc);
	printf("rs485 recv:          %s /dev/ttyACM0 1 9600 8 1 N 0 3 255\n", proc);
}

int main(int argc, char **argv)
{
	int fd, ret, baudrate=9600, bits=8, stop=1, direction=1, recv_len, send_len, rs485=1, flen, flow_ctrl=0, max_tty_len=255;
	char parity = 'N';
	char* dev_name;
	struct timeval begin_tv, end_tv;
	float interval_tv;

	if(argc < 10) {
        usage(argv[0]);
		return -1;
	}

	dev_name = argv[1];
	rs485 = atoi(argv[2]);
	baudrate = atoi(argv[3]);
	bits = atoi(argv[4]);
	stop = atoi(argv[5]);
	parity = argv[6][0];
	flow_ctrl = atoi(argv[7]);
	direction = atoi(argv[8]);

	if(argc == 10) {
		max_tty_len = atoi(argv[9]);
	}

	printf("dev_name %s, rs485 %d, baudrate %d, bits %d, stop %d, parity %c, flow_ctrl %d, direction %d, max_tty_len %d\n",
	dev_name, rs485, baudrate, bits, stop, parity, flow_ctrl, direction, max_tty_len);

	fd = tty_open(dev_name);
	if(fd < 0) {
		printf("open error\n");
		return -1;
	}

	ret	= tty_set_opt(fd, rs485, baudrate, bits, stop, flow_ctrl, parity);
	if (ret < 0) {
		printf("Set Port Error\n");
		goto _close;
	}

	alloc_buf();
	if((flen = file2buf("send.dat", write_buf, MAX_BUF_SIZE)) <= 0) {
		printf("file2buf fail!\n");
		goto _close;
	}

	while(1) {
		memset(read_buf, 0, MAX_BUF_SIZE);
		recv_len = 0, send_len = 0;

		if(direction == 0) {
			gettimeofday(&begin_tv, NULL);
			send_len = tty_send(fd, write_buf, flen);
			recv_len = tty_recv(fd, read_buf, max_tty_len);
			gettimeofday(&end_tv, NULL);
			interval_tv = 1000000*(end_tv.tv_sec-begin_tv.tv_sec) + (end_tv.tv_usec-begin_tv.tv_usec);
			printf("[%fus] send %d recv %d\n", interval_tv, send_len, recv_len);
			usleep(500000);

		} else if (direction == 1) {
			gettimeofday(&begin_tv, NULL);
			recv_len = tty_recv(fd, read_buf, max_tty_len);
			usleep(500000);
			send_len = tty_send(fd, read_buf, recv_len);
			gettimeofday(&end_tv, NULL);
			interval_tv = 1000000*(end_tv.tv_sec-begin_tv.tv_sec) + (end_tv.tv_usec-begin_tv.tv_usec);
			printf("[%fus] recv %d send %d\n", interval_tv, recv_len, send_len);
		} else if (direction == 2) {
			gettimeofday(&begin_tv, NULL);
			send_len = tty_send(fd, write_buf, flen);
			gettimeofday(&end_tv, NULL);
			interval_tv = 1000000*(end_tv.tv_sec-begin_tv.tv_sec) + (end_tv.tv_usec-begin_tv.tv_usec);
			printf("[%fus] send %d\n", interval_tv, send_len);
		} else if (direction == 3) {
			gettimeofday(&begin_tv, NULL);
			recv_len = tty_recv(fd, read_buf, MAX_BUF_SIZE);
			gettimeofday(&end_tv, NULL);
			interval_tv = 1000000*(end_tv.tv_sec-begin_tv.tv_sec) + (end_tv.tv_usec-begin_tv.tv_usec);
			printf("[%fus] recv %d\n", interval_tv, recv_len);
			usleep(500000);
		}
	}
_close:
	tty_close(fd);
	free_buf();
	return 0;
}

