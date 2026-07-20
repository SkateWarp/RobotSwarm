# Aceptación multiusuario de producción

Estos dos arneses comprueban el flujo público de RobotSwarm sin reutilizar una
sesión histórica. `robotswarm-prod-e2e.py` valida la API, el aislamiento HLS y
la limpieza de dos cuentas. `robotswarm-visible-e2e.py` repite el recorrido en
dos ventanas normales de Chrome, con GPU habilitada, interacción real y
capturas sanitizadas. El segundo arnés no admite modo headless.

## Archivos locales requeridos

Las credenciales se leen, sin ejecutar el archivo como código, desde
`/tmp/robotswarm-e2e-credentials.env`. Debe pertenecer al usuario, tener modo
`0600` y contener exactamente los marcadores, correos y contraseñas de las dos
cuentas de prueba mediante las claves `TEST_A_ID`, `TEST_A_EMAIL`,
`TEST_A_PASSWORD`, `TEST_B_ID`, `TEST_B_EMAIL` y `TEST_B_PASSWORD`.

El enlace entre ambos ensayos utiliza `/tmp/robotswarm-e2e-binding.key`, un
archivo regular de 32 bytes, también `0600`. Es una clave aleatoria independiente
de las contraseñas y nunca se incorpora al reporte. Puede generarse una sola vez
para la pareja de ejecuciones con:

```bash
umask 077
openssl rand -out /tmp/robotswarm-e2e-binding.key 32
```

El operador debe comprobar primero en los hosts que backend y worker GPU
corresponden al mismo SHA completo. Ese valor se pasa explícitamente a los dos
programas; no se acepta una revisión abreviada.

## Orden de ejecución

La planificación comprueba permisos y esquema sin acceder a la red:

```bash
python3 scripts/acceptance/robotswarm-prod-e2e.py --plan
```

Después de autorizar la prueba de producción, se ejecuta la parte API y se
conserva su salida en un directorio del propietario:

```bash
python3 scripts/acceptance/robotswarm-prod-e2e.py \
  --execute-production \
  --deployment-commit <sha-completo> \
  --output /tmp/robotswarm-acceptance/api.json
```

El reporte aprobado tiene una vigencia de treinta minutos para iniciar la parte
visual. En WSL, `--profile-root` debe apuntar a un directorio temporal de Windows
que pertenezca al operador:

```bash
python3 scripts/acceptance/robotswarm-visible-e2e.py \
  --api-report /tmp/robotswarm-acceptance/api.json \
  --deployment-commit <sha-completo> \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output-dir /tmp/robotswarm-acceptance/visual
```

La prueba visual exige dos rosters distintos —tres y siete robots por defecto—,
HLS privado en ambas cuentas, FPS numéricos, clic, arrastre, rueda, teclado,
pantalla completa, tareas concurrentes y continuidad de B después de detener A.
Todos los JSON y PNG se crean con modo `0600`; los perfiles de Chrome, sesiones
y controles interactivos se liberan incluso ante interrupción. Un resultado no
se considera aprobatorio si la limpieza falla.

Los programas muestran únicamente mensajes y reportes saneados. No deben
copiarse a la documentación el archivo de credenciales, la clave de enlace, los
perfiles del navegador ni las respuestas HTTP crudas.
