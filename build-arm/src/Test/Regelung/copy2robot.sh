#!/bin/bash
. lib.sh.in

setc remote_user "root"
setc remote_host "es139.ntb.ch"
setc remote_dir "einbein/03_Test_Regelung"

scp Trajektorie/Trajektorie $remote_user@$remote_host:$remote_dir
scp Fusspunkt/Fusspunkt $remote_user@$remote_host:$remote_dir
scp Controller/Controller $remote_user@$remote_host:$remote_dir
