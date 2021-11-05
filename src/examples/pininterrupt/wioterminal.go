//go:build wioterminal
// +build wioterminal

package main

import "machine"

const (
	buttonPin       = machine.BUTTON
	buttonMode      = machine.PinInput
	buttonPinChange = machine.PinFalling
)
