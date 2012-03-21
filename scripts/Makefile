

all : install.img netboot.img

clean : 
	rm -f install.img netboot.img

%.img : %
	../tools/mkimage -T script -C none -n $^ -d $^ $@

