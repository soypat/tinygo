//go:build stm32
// +build stm32

package main

import "machine"

const (
	buttonPin       = machine.BUTTON
	buttonMode      = machine.PinInputPulldown
	buttonPinChange = machine.PinRising | machine.PinFalling
)
