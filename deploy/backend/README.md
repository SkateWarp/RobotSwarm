# Recuperación del backend después del arranque

El backend y MediaMTX publican puertos sobre la IPv4 concreta de la VM. Docker
puede iniciar antes de que DHCP asigne esa dirección y, en ese caso, no vuelve
a intentar un contenedor que nunca llegó a arrancar correctamente.

`recover-production-startup.sh` actúa solamente sobre los tres contenedores ya
creados por el despliegue. No ejecuta `docker compose up`, no reconstruye
imágenes y no lee archivos de variables o secretos. Antes de iniciar algo:

1. comprueba los labels de proyecto y servicio de `db_prod`, `media_prod` y
   `backend_prod`;
2. obtiene las IPv4 desde los bindings existentes de `8554/tcp` y
   `44336/tcp`;
3. espera que esas direcciones estén asignadas al host;
4. inicia únicamente contenedores en estado `created` o `exited`, en el orden
   base de datos, media y backend; y
5. exige estado `healthy` cuando el contenedor declara un healthcheck.

El código 75 representa una condición transitoria y permite que systemd vuelva
a intentarlo. El código 78 representa una configuración inesperada y evita un
bucle ante labels o bindings incorrectos. Si `docker inspect` falla, la
ausencia de un contenedor sólo se considera configuración inválida después de
confirmarla con un listado exitoso del daemon. Un error ambiguo conserva el
código 75.

## Prueba offline

La prueba usa dobles locales de `docker` e `ip`; no necesita el daemon ni
modifica la red:

```bash
python3 deploy/backend/test-recover-production-startup.py
bash -n deploy/backend/recover-production-startup.sh
```

## Instalación en la VM

La instalación es una operación administrativa explícita y no forma parte del
workflow de GitHub:

```bash
sudo install -m 0755 \
  deploy/backend/recover-production-startup.sh \
  /usr/local/sbin/robotswarm-recover-production-startup
sudo install -m 0644 \
  deploy/backend/robotswarm-production-startup-recovery.service \
  /etc/systemd/system/robotswarm-production-startup-recovery.service
sudo systemctl daemon-reload
sudo systemctl enable robotswarm-production-startup-recovery.service
sudo systemctl start robotswarm-production-startup-recovery.service
```

El perfil cableado debe requerir IPv4, aunque esa opción por sí sola no cubre
un DHCP que tarde más que el timeout de `NetworkManager-wait-online`:

```bash
sudo nmcli connection modify "Wired connection 1" ipv4.may-fail no
```

No se debe reactivar la conexión durante una sesión SSH solo para aplicar este
valor; NetworkManager lo utilizará en la siguiente activación. Para revisar la
recuperación:

```bash
systemctl status robotswarm-production-startup-recovery.service
journalctl -u robotswarm-production-startup-recovery.service
docker ps --filter name=_prod
```

Si la unidad se retira, primero se deshabilita y después se eliminan sus dos
archivos instalados:

```bash
sudo systemctl disable --now robotswarm-production-startup-recovery.service
sudo rm -f \
  /etc/systemd/system/robotswarm-production-startup-recovery.service \
  /usr/local/sbin/robotswarm-recover-production-startup
sudo systemctl daemon-reload
```
