#
# Top level makefile for documentation build
#

BUILDDIR ?= doc/_build
DOC_TAG ?= development
SPHINXOPTS ?= -q
KCONFIG_TURBO_MODE ?= 0
DUMMY_BUILD ?= OFF

# Documentation targets
# ---------------------------------------------------------------------------

.PHONY: clean htmldocs htmldocs-fast pdfdocs doxygen

htmldocs-fast:
	${MAKE} htmldocs KCONFIG_TURBO_MODE=1

htmldocs pdfdocs doxygen: configure
	cmake --build ${BUILDDIR} -- $@  # -v # VERBOSE=1

# Run CMake every time cause it's quick and re-configures TURBO_MODE if
# needed
.PHONY: configure
configure:
	cmake -GNinja  -B${BUILDDIR} -Sdoc/ -DDOC_TAG=${DOC_TAG} \
		-DSPHINXOPTS=${SPHINXOPTS} \
		-DKCONFIG_TURBO_MODE=${KCONFIG_TURBO_MODE} \
		-DDUMMY_BUILD=${DUMMY_BUILD}

clean:
	rm -rf ${BUILDDIR}
