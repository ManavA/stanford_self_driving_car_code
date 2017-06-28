all: installed

TARBALL     = build/flann-1.2-src.zip
TARBALL_URL = http://pr.willowgarage.com/downloads/flann-1.2-src.zip
UNPACK_CMD  = unzip -x
SOURCE_DIR  = build/flann-1.2-src
TARBALL_PATCH = cmakelists.patch dist_extended.patch

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	echo "Building FLANN..."
	mkdir -p build/build && cd build/build && cmake ../flann-1.2-src/src && make && make install
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find FLANN`/build/flann-1.2-src/build/lib/libflann.dylib build/flann-1.2-src/build/lib/libflann.dylib; \
	fi
	touch installed

clean: wipe

wiped: Makefile
	make wipe
	touch wiped

wipe:
	rm -rf build installed
