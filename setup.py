#!/usr/bin/env python

# Copyright (c) 2010, Nicolas Trangez
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the <organization> nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import os.path

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

# This doesn't work since the pylibpci package module imports _libpci, which
# doesn't exist while buiding.
# Fallback to 'manual' parsing.
#import pylibpci

def find_version():
    fd = open(os.path.join(os.path.dirname(__file__), 'pylibpci',
        '__init__.py'))

    try:
        for line in fd.xreadlines():
            if not line.startswith('__version__ ='):
                continue

            version_expr = line.split('=', 1)[1].strip()
            version = eval(version_expr)

            return version

    finally:
        fd.close()

    return (0, 0, 1)


_libpci = Extension(
    'pylibpci._libpci', ['pylibpci/_libpci.pyx', ],
    libraries=['pci', ],
    extra_compile_args=['-Wall', '-Werror', ],
    extra_link_args=['-Wall', '-Werror', ],
)

setup(
    name='pylibpci',
    version='.'.join(str(i) for i in find_version()),
    description='Python bindings for libpci',

    author='Nicolas Trangez',
    author_email='eikke@eikke.com',

    packages=['pylibpci', ],

    cmdclass={'build_ext': build_ext, },

    ext_modules=[_libpci, ],
)
