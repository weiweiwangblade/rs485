rs485:rs485.o
	gcc rs485.o -o rs485
rs485.o:rs485.c
	gcc -c rs485.c -o rs485.o
clean:
	rm -rf *.o rs485

