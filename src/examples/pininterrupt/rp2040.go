//go:build rp2040
// +build rp2040

package main

import "machine"

const (
	buttonPin       = machine.GPIO5 // GP5 on Pico and D10 on Nano-RP2040
	buttonMode      = machine.PinInputPullup
	buttonPinChange = machine.PinRising
)
