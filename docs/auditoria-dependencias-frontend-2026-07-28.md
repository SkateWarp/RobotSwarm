# Auditoría de dependencias y calidad del frontend

## 1. Objetivo y alcance

Esta revisión se realizó después de recuperar producción. Su objetivo fue
distinguir entre avisos de `npm audit` realmente alcanzables desde las
pantallas de RobotSwarm y dependencias heredadas que permanecen en el árbol,
pero no participan en la aplicación desplegada.

No se aplicó `npm audit fix --force`. Esa operación proponía cambios
incompatibles, entre ellos versiones impropias de `react-scripts` y
`rosreact`. Tampoco se ejecutó un autofix global de ESLint porque habría
mezclado correcciones reales con miles de cambios de formato.

## 2. Resultados reproducibles

La auditoría de `SwarmFrontend` informó:

| Severidad | Paquetes |
| --- | ---: |
| Baja | 17 |
| Moderada | 26 |
| Alta | 51 |
| Crítica | 5 |
| **Total** | **99** |

Con `--omit=dev` permanecieron 97. Esta cifra no equivale a 97 fallos
explotables en producción: `react-scripts` y otras herramientas de compilación
están declaradas como dependencias normales, aunque Cloudflare sólo recibe el
bundle estático.

El `package.json` de la raíz añadió ocho avisos, todos vinculados a un `gzweb`
que no está importado por la aplicación ni por los workflows.

El lint completo produjo 206 errores y 11.003 advertencias en 51 archivos.
De las advertencias, 10.931 eran únicamente diferencias de Prettier. Las seis
superficies activas —Control, Historial, Plantillas, Robots, Grupos y
Usuarios— aprobaron el lint focal usado por CI; la mayor parte del ruido se
encuentra en variantes Fuse, Fraga, Baldom o componentes sustituidos.

## 3. Interpretación de los avisos críticos

| Dependencia | Procedencia | Alcance observado |
| --- | --- | --- |
| `fast-xml-parser` | `gzweb` | `gzweb` no se importa ni aparece en el bundle |
| `protobufjs` | `gzweb` y Firestore | No se usa Firestore; `gzweb` está inactivo |
| `shell-quote` | `react-scripts` | Herramienta de build, no runtime del navegador |
| `form-data` | Auth0 y jsdom | Auth0 está sin configurar; jsdom se usa en pruebas |
| `websocket-driver` | Firebase y webpack-dev-server | Firebase Database no se usa; dev-server no se despliega |

Los cinco avisos merecen limpieza, pero no representan cinco rutas remotas
confirmadas en el sitio actual.

## 4. Dependencias activas que requieren atención

### 4.1 Axios

`axios@0.26.0` participa en autenticación y llamadas al API. La URL base es
constante y no se encontró una ruta que acepte destinos arbitrarios del
usuario, lo que reduce la aplicabilidad de varios avisos SSRF del adaptador
Node. Aun así, la migración a Axios 1 debe tratarse como una entrega focal con
pruebas de login, refresh, CRUD, sesiones y manejo de errores.

### 4.2 Lodash, Moment y React Router

Se observaron versiones antiguas de estas tres bibliotecas:

- `lodash@4.17.21`: no se usa `_.template`, que es la superficie del aviso
  relevante;
- `moment@2.29.1`: no se encontró cambio de locale con entrada externa; y
- `react-router-dom@6.2.2`: las rutas enviadas a navegación son constantes o
  proceden del inventario interno.

Son actualizaciones razonables dentro de la misma familia, pero deben
validarse en un PR frontend separado.

### 4.3 Firebase, Auth0 y gzweb

RobotSwarm usa JWT propio y el visor vigente usa HLS/WHEP/VNC. Firebase y Auth0
son restos de la plantilla. `gzweb` sólo conserva una declaración TypeScript.
Actualizar `gzweb` no es una solución apropiada: su versión reciente requiere
Node 24/npm 11, mientras CI usa Node 18, y conserva dependencias vulnerables.
La acción recomendada es retirar estas rutas muertas.

## 5. Plan recomendado

Para mantener revisiones pequeñas y reducir el riesgo de regresión:

1. retirar `gzweb`, `rosreact`, Auth0 y Firebase junto con sus módulos
   huérfanos;
2. eliminar el manifiesto raíz si se confirma que sólo contiene herramientas
   sin uso;
3. actualizar Lodash, Moment, Prism y React Router dentro de versiones
   compatibles;
4. migrar Axios 0 → 1 en un PR focal;
5. abordar ECharts 5 → 6 y CRA → Vite como migraciones independientes; y
6. ejecutar ESLint por carpeta después de borrar código muerto, sin un
   `--fix` global.

La validación mínima de cada entrega frontend debe incluir `npm ci`, las 182
pruebas actuales, build de producción, lint de los archivos modificados y una
aceptación visible de login, secciones y visor.

## 6. Conclusión

La aplicación activa no presenta evidencia de explotación de los cinco avisos
críticos, pero el árbol de dependencias contiene deuda significativa y eleva
el costo de futuras actualizaciones. La prioridad no es forzar versiones de
forma automática, sino eliminar dependencias muertas y después migrar las
bibliotecas realmente utilizadas con pruebas focales.
