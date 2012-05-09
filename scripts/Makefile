

all :netboot.img reboot.img

clean : 
	rm -f install.img netboot.img reboot.img

%.img : %
	../tools/mkimage -T script -C none -n $^ -d $^ $@ ;\
	

