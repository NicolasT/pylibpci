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

cdef extern from "pci/pci.h":
    ctypedef int u8
    ctypedef int u16
    ctypedef int u32

    ctypedef int pciaddr_t

    cdef enum:
        PCI_LIB_VERSION

    cdef enum pci_access_type:
        PCI_ACCESS_AUTO
        PCI_ACCESS_SYS_BUS_PCI
        PCI_ACCESS_PROC_BUS_PCI
        PCI_ACCESS_I386_TYPE1
        PCI_ACCESS_I386_TYPE2
        PCI_ACCESS_FBSD_DEVICE
        PCI_ACCESS_AIX_DEVICE
        PCI_ACCESS_NBSD_LIBPCI
        PCI_ACCESS_OBSD_DEVICE
        PCI_ACCESS_DUMP
        PCI_ACCESS_MAX

    # Forward-declaration
    cdef struct pci_dev

    cdef struct pci_access:
        unsigned int method
        int writable
        int buscentric

        char *id_file_name
        int free_id_name
        int numeric_ids

        unsigned int id_lookup_mode

        int debugging

        void (*error)(char *msg, ...)
        void (*warning)(char *msg, ...)
        void (*debug)(char *msg, ...)

        pci_dev *devices

    cdef pci_access *pci_alloc()
    cdef void pci_init(pci_access *)
    cdef void pci_cleanup(pci_access *)

    cdef void pci_scan_bus(pci_access *)
    cdef pci_dev *pci_get_dev(pci_access *, int, int, int, int)
    cdef void pci_free_dev(pci_dev *)

    cdef int pci_lookup_method(char *)
    cdef char *pci_get_method(int)

    cdef struct pci_param:
        pci_param *next
        char *param
        char *value
        char *help

    cdef char *pci_get_param(pci_access *, char *)
    cdef int pci_set_param(pci_access *, char *, char *)
    cdef pci_param *pci_walk_params(pci_access *, pci_param *)

    # Forward-declaration
    cdef struct pci_cap:
        pass

    cdef struct pci_dev:
        pci_dev *next
        u16 domain
        u8 bus
        u8 dev
        u8 func

        int known_fields
        u16 vendor_id
        u16 device_id
        u16 device_class
        int irq
        pciaddr_t base_addr[6]
        pciaddr_t size[6]
        pciaddr_t rom_base_addr
        pciaddr_t rom_size
        pci_cap *first_cap
        char *phy_slot

    cdef enum:
        PCI_ADDR_IO_MASK
        PCI_ADDR_MEM_MASK
        PCI_ADDR_FLAG_MASK

    cdef u8 pci_read_byte(pci_dev *, int)
    cdef u16 pci_read_word(pci_dev *, int)
    cdef u32 pci_read_long(pci_dev *, int)
    cdef int pci_read_block(pci_dev *, int, u8, int)
    cdef int pci_read_vpd(pci_dev *, int, u8, int)
    cdef int pci_write_byte(pci_dev *, int, u8)
    cdef int pci_write_word(pci_dev *, int, u16)
    cdef int pci_write_long(pci_dev *, int, u32)
    cdef int pci_write_block(pci_dev *, int, u8, int)

    cdef int pci_fill_info(pci_dev *, int)

    cdef enum:
        PCI_FILL_IDENT
        PCI_FILL_IRQ
        PCI_FILL_BASES
        PCI_FILL_ROM_BASE
        PCI_FILL_SIZES
        PCI_FILL_CLASS
        PCI_FILL_CAPS
        PCI_FILL_EXT_CAPS
        PCI_FILL_PHYS_SLOT
        PCI_FILL_RESCAN

    cdef void pci_setup_cache(pci_dev *, u8 *, int)

    cdef struct pci_cap:
        pci_cap *next
        u16 id
        u16 type
        unsigned int addr

    cdef enum:
        PCI_CAP_NORMAL
        PCI_CAP_EXTENDED

    cdef pci_cap *pci_find_cap(pci_dev *, unsigned int id, unsigned int type)

    cdef struct pci_filter:
        int domain
        int bus
        int slot
        int func

        int vendor
        int device

    cdef void pci_filter_init(pci_access *, pci_filter *)
    cdef char *pci_filter_parse_slot(pci_filter *, char *)
    cdef char *pci_filter_parse_id(pci_filter *, char *)
    cdef int pci_filter_match(pci_filter *, pci_dev *)

    cdef char *pci_lookup_name(pci_access *, char *, int, int, ...)

    cdef int pci_loadname_list(pci_access *)
    cdef void pci_free_name_list(pci_access *)
    cdef void pci_set_name_list_path(pci_access *, char *, int)
    cdef void pci_id_cache_flush(pci_access *)

    cdef enum pci_lookup_mode:
        PCI_LOOKUP_VENDOR
        PCI_LOOKUP_DEVICE
        PCI_LOOKUP_CLASS
        PCI_LOOKUP_SUBSYSTEM
        PCI_LOOKUP_PROGIF
        PCI_LOOKUP_NUMERIC
        PCI_LOOKUP_NO_NUMBERS
        PCI_LOOKUP_MIXED
        PCI_LOOKUP_NETWORK
        PCI_LOOKUP_SKIP_LOCAL
        PCI_LOOKUP_CACHE
        PCI_LOOKUP_REFRESH_CACHE


    cdef enum:
        PCI_INTERRUPT_LINE
        PCI_INTERRUPT_PIN
        PCI_MIN_GNT
        PCI_MAX_LAT


class PCIDevice(object):
    def __init__(self, domain, bus, dev, func, vendor_id, device_id,
            device_class, irq, interrupt_pin, base_addr,
            vendor_name, device_name, device_class_name):
        self.domain = domain
        self.bus = bus
        self.dev = dev
        self.func = func
        self.vendor_id = vendor_id
        self.device_id = device_id
        self.device_class = device_class
        self.irq = irq
        self.interrupt_pin = interrupt_pin
        self.base_addr = base_addr

        self.vendor_name = vendor_name
        self.device_name = device_name
        self.device_class_name = device_class_name

    def __str__(self):
        return '%s: %s %s' % (self.device_class_name, self.vendor_name,
                self.device_name)


def list_devices():
    return tuple(_list_devices())

cdef _list_devices():
    result = []

    cdef unsigned int c
    cdef char vendor_namebuf[1024], *vendor_name
    cdef char device_namebuf[1024], *device_name
    cdef char class_namebuf[1024], *class_name

    cdef pci_access *pacc
    cdef pci_dev *dev

    pacc = pci_alloc()
    pci_init(pacc)

    pci_scan_bus(pacc)

    dev = pacc.devices
    while dev != NULL:
        pci_fill_info(dev, PCI_FILL_IDENT | PCI_FILL_BASES | PCI_FILL_CLASS)

        c = pci_read_byte(dev, PCI_INTERRUPT_PIN)

        vendor_name = pci_lookup_name(pacc, vendor_namebuf,
                sizeof(vendor_namebuf), PCI_LOOKUP_VENDOR, dev.vendor_id)
        device_name = pci_lookup_name(pacc, device_namebuf,
                sizeof(device_namebuf), PCI_LOOKUP_DEVICE, dev.vendor_id,
                dev.device_id)
        class_name = pci_lookup_name(pacc, class_namebuf, sizeof(class_namebuf),
                PCI_LOOKUP_CLASS, dev.device_class)

        dev_ = PCIDevice(dev.domain, dev.bus, dev.dev, dev.func, dev.vendor_id,
                dev.device_id, dev.device_class, dev.irq, c, dev.base_addr[0],
                vendor_name, device_name, class_name)
        result.append(dev_)

        dev = dev.next

    pci_cleanup(pacc)

    return result
