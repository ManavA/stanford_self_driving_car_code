############################################################
# Makefile template for wrapping external libraries into ROS
# author: Stanford Driving
############################################################

# name and build dir
LIBNAME = CGAL-3.7
SOURCE_DIR = build/$(LIBNAME)

# download and extract
TARBALL = build/$(LIBNAME).tar.gz
TARBALL_URL = https://gforge.inria.fr/frs/download.php/27641/$(LIBNAME).tar.gz
# MD5SUM_FILE = $(LIBNAME).tar.gz.md5sum
# UNPACK_CMD = tar xzf

# configure
# CONFIGURE_CMD = chmod +x $(SOURCE_DIR)/configure && cd $(SOURCE_DIR) && ./configure --prefix=$(PWD)
CONFIGURE_CMD = cd $(SOURCE_DIR) && cmake -DCMAKE_INSTALL_PREFIX=$(PWD) .

# build
BUILD_CMD = cd $(SOURCE_DIR) && make

# install
INSTALL_CMD = cd $(SOURCE_DIR) && make install


############################################################

all: installed cleanup

include $(shell rospack find mk)/download_unpack_build.mk

configured: $(SOURCE_DIR)/unpacked
	@echo "configuring $(LIBNAME)"
	$(CONFIGURE_CMD)
	@touch configured
	@echo "configuring $(LIBNAME) finished"

built: configured
	@echo "building $(LIBNAME)"
	$(BUILD_CMD)
	@touch built
	@echo "building $(LIBNAME) finished"

installed: built 
	@echo "installing $(LIBNAME)"
	$(INSTALL_CMD)
	@touch installed
	@echo "installing $(LIBNAME) finished"

cleanup: 
	@echo "cleaning temporary build files"
	-rm -rf build
	@mkdir -p $(SOURCE_DIR)
	@touch $(SOURCE_DIR)/unpacked
	@touch configured
	@touch built
	@touch installed

clean:
	-rm -rf build configured built installed

wipe: clean
	-rm -rf include lib
