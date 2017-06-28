############################################################
# Makefile template for wrapping external libraries into ROS
############################################################

# name and build dir
LIBNAME = qpOASES-2.0
SOURCE_DIR = build/$(LIBNAME)

# download and extract
TARBALL = build/$(LIBNAME).tar.gz
#TARBALL_URL = http://www.kuleuven.be/optec/software/qpOASES/download/$(LIBNAME).tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/$(LIBNAME).tar.gz
TARBALL_PATCH = patchfile
# MD5SUM_FILE = $(LIBNAME).tar.gz.md5sum
# UNPACK_CMD = tar xzf

# configure
CONFIGURE_CMD =

# build
BUILD_CMD = cd $(SOURCE_DIR)/SRC && make -f Makefile

# install
INSTALL_CMD = mkdir -p include && cp -rf $(SOURCE_DIR)/INCLUDE include/qpOASES && mkdir -p lib && cp -rf $(SOURCE_DIR)/SRC/*.so lib


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
