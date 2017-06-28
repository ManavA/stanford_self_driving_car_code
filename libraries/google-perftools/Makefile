TARBALL = build/google-perftools-1.5.tar.gz
TARBALL_URL = http://google-perftools.googlecode.com/files/google-perftools-1.5.tar.gz
SOURCE_DIR = build/google-perftools-1.5
#TARBALL_PATCH = patchfile

all: installed

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && chmod +x configure && ./configure --enable-frame-pointers && make
	mkdir -p bin
	cp $(SOURCE_DIR)/src/pprof bin/ #Can't symlink or rosrun won't accept it.
	touch $@

clean: 
	-rm -rf $(SOURCE_DIR) installed bin/

wipe: clean
	-rm -rf build