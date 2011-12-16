

all : flashem.img netboot.img

clean : 
	rm -f flashem.img netboot.img

%.img : %
	../tools/mkimage -T script -C none -n $^ -d $^ $@

