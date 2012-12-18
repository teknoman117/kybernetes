#!/bin/bash

# Handle odd bug
ip link set $1 down
ip link set $1 up

# Bring down the connection
ip link set $1 down

# Configure wireless settings
iwconfig $1 mode ad-hoc
iwconfig $1 channel 4
iwconfig $1 essid 'kybernetes'
iwconfig $1 key 1234567890

# Bring Link back up
ip link set $1 up

# Start the zeroconf stack on the thing
avahi-autoipd -D $1
