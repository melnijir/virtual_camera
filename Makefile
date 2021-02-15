
.PHONY: module utility

all: module utility

module: bin bin/vcmod.ko bin/vcctrl

bin: 
	mkdir bin

bin/vcmod.ko:
	cd module; $(MAKE) ; cd ..; cp module/vcmod.ko bin/

bin/vcctrl:
	cd utils; $(MAKE); cd ..; cp utils/vcctrl bin/

clean:
	rm -r bin; cd module; $(MAKE) clean; cd ../utils; $(MAKE) clean;
