#!/bin/bash
# Iniciar Apache en segundo plano
apachectl start
# Ejecutar el script de Python
cd /usr/local/apache2/htdocs/

exec python3 grabber.py
