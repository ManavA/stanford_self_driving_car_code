all: installed

TARBALL     = build/ann_1.1.1.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/ann_1.1.1.tar.gz
UNPACK_CMD  = tar xfz
SOURCE_DIR  = build/ann_1.1.1
TARBALL_PATCH = gcc43_shared.patch fPIC.patch distance.patch

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked Makefile
	echo "Building ANN..."
	if test `uname` = Darwin; then \
	  cd build/ann_1.1.1 && make macosx-g++; \
	else\
	  cd build/ann_1.1.1 && make linux-shared-g++; \
	fi
	touch installed
clean: wipe
wipe:
	rm -rf build installed
