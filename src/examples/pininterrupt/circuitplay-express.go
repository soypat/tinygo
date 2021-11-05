//go:build circuitplay_express
// +build circuitplay_express

package main

import "machine"

const (
	buttonPin       = machine.BUTTON
	buttonMode      = machine.PinInputPulldown
	buttonPinChange = machine.PinFalling
)
