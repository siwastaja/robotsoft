#!/bin/bash

user="pulu"
hostdir="/home/${user}/robotsoft"

while true
do
	sleep 4
	mv ${hostdir}/log.3.txt ${hostdir}/log.4.txt
	mv ${hostdir}/log.2.txt ${hostdir}/log.3.txt
	mv ${hostdir}/log.1.txt ${hostdir}/log.2.txt
	mv ${hostdir}/log.txt ${hostdir}/log.1.txt
	touch ${hostdir}/rn1host.lock
	${hostdir}/robotsoft > ${hostdir}/log.txt
	HOSTRET=$?
	rm -f ${hostdir}/rn1host.lock

	echo "robotsoft returned ${HOSTRET}"

	case "$HOSTRET" in
	135)	echo "Rebooting the computer..."
		sudo shutdown -r now
		;;

	136)	echo "Shutting down the computer..."
		sudo shutdown -h now
		;;

	5)	echo "Quitting..."
		break
		;;

	*)	echo "Restarting rn1host..."
		;;
	esac
done
