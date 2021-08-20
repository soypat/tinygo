// +build rp2040

package machine

import (
	"device/arm"
	"device/rp"
	"runtime/interrupt"
	"runtime/volatile"
	"unsafe"
)

const (
	// Number of spin locks available
	_NUMSPINLOCKS = 32
	// Number of interrupt handlers available
	_NUMIRQ               = 32
	_PICO_SPINLOCK_ID_IRQ = 9
)

// Enable or disable a specific interrupt on the executing core.
// num is the interrupt number which must be in [0,31].
func irqSet(num uint32, enabled bool) {
	if num >= _NUMIRQ {
		return
	}
	irqSetMask(1<<num, enabled)
}

// pico-sdk: All interrupts handlers should be of this type, and follow normal ARM EABI register saving conventions
type irqHandler func()

type spinLock struct {
	reg *volatile.Register32
}

// Initializes spinlock to a spin lock number.
//go:inline
func (lock *spinLock) init(num uint32) {
	const SIO_Offset, SIO_SPINLOCK0_OFFSET uintptr = 0xd0000000, 0x00000100
	if num >= _NUMSPINLOCKS {
		panic("spinlocknum >= 32")
	}
	lock.reg = (*volatile.Register32)(unsafe.Pointer(SIO_Offset + SIO_SPINLOCK0_OFFSET + uintptr(num)*4))
}

// Release a spin lock without re-enabling interrupts
//go:inline
func (lock spinLock) unlock(is interrupt.State) {
	memFenceRelease()
	lock.reg.Set(0)
	interrupt.Restore(is)
}

// Acquire a spin lock safely with interrupts.
//go:inline
func (lock spinLock) lock() interrupt.State {
	is := interrupt.Disable()
	// Note we don't do a wfe or anything, because by convention these spin_locks are VERY SHORT LIVED and NEVER BLOCK and run
	// with INTERRUPTS disabled (to ensure that)... therefore nothing on our core could be blocking us, so we just need to wait on another core
	// anyway which should be finished soon
	for lock.reg.Get() != 0 {
	}
	memFenceAcquire()
	return is
}

func irqSetExclusiveHandler(num uint32, handler irqHandler) {
	if num >= _NUMIRQ {
		return
	}
	lock := spinLock{}
	lock.init(_PICO_SPINLOCK_ID_IRQ)
	is := lock.lock()
	// current := getVtableHandler(num)
	// C SDK asserts either handler is unset or is same handler
	// hard_assert(current == __unhandled_user_irq || current == handler);
	setHandler(num, handler)
	lock.unlock(is)
}

func setHandler(num uint32, handler irqHandler) {
	// update vtable (vtable_handler may be same or updated depending on cases, but we do it anyway for compactness)
	getVtable()[16+num] = handler
}

func irqSetMask(mask uint32, enabled bool) {
	if enabled {
		// Clear pending before enable
		// (if IRQ is actually asserted, it will immediately re-pend)
		rp.PPB.NVIC_ICPR.Set(mask)
		rp.PPB.NVIC_ISER.Set(mask)
	} else {
		rp.PPB.NVIC_ICER.Set(mask)
	}
}

type armv6SCB struct {
	cpuid volatile.Register32 // Read-Only
	icsr  volatile.Register32
	vtor  volatile.Register32
	aircr volatile.Register32
	scr   volatile.Register32
}

var (
	// SCB == System Control Block
	SCB = (*armv6SCB)(unsafe.Pointer(uintptr(0xe0000000 + 0x0000ed00))) // PPBBase + PPB_CPUID Offset
)

// Equivalent to get_vtable()[16+num] from pico-sdk.
func getVtableHandler(num uint32) irqHandler {
	if num >= _NUMIRQ {
		return nil
	}
	tablePtr := getVtable()
	return tablePtr[num+16]
}

func getVtable() *[16 + _NUMIRQ]irqHandler {
	// abuse of unsafe gets us there.
	return (*[16 + _NUMIRQ]irqHandler)(unsafe.Pointer(uintptr(SCB.vtor.Get())))
}

// The DMB (data memory barrier) acts as a memory barrier, all memory accesses prior to this
// instruction will be observed before any explicit access after the instruction.
//go:inline
func memFenceAcquire() { arm.Asm("dmb") }

//go:inline
func memFenceRelease() { arm.Asm("dmb") }
