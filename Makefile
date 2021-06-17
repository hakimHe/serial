all:
	$(CC) linux_tty.c -o linux_tty

clean:
	rm linux_tty
