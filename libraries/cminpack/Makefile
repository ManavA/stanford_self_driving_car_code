all: installed

TARBALL     = build/cminpack-1.0.2.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/cminpack-1.0.2.tar.gz
UNPACK_CMD  = tar xfz
SOURCE_DIR  = build/cminpack-1.0.2
TARBALL_PATCH = fPIC.patch

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked Makefile
	echo "Building CMinpack..."
	cd $(SOURCE_DIR) && make
	touch installed
clean: wipe
wipe:
	rm -rf build installed
