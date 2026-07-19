# README: Proyecto de Enjambre Robótico https://rs.zerav.la/login

The React frontend is hosted by Cloudflare Workers and deployed from GitHub.
The .NET API remains available separately at `https://robot.zerav.la`.
Every successful `main` CI revision is deployed to the backend VM; this avoids
missing backend changes in pushes that contain more than one commit. GPU worker
releases use a separate, manually approved maintenance workflow. That workflow
acquires a backend drain lease before it tests or replaces the worker, so new
sessions cannot enter the host during the deployment window.

## Estado y documentación

El [estado de implementación](IMPLEMENTATION_STATUS.md) distingue entre código
integrado, pruebas visibles ya realizadas y la aceptación final que todavía
requiere despliegue. El
[informe técnico de comisionamiento](docs/informe-comisionamiento-final.md)
mantiene en español la bitácora académica de incidencias, diagnóstico,
correcciones, capturas y resultados. La arquitectura operativa actual se resume
en el [plan de control](docs/swarm-control-plan.md).

## Objetivos

1. Desarrollar un sistema de enjambre robótico escalable y eficiente.
2. Facilitar la colaboración entre robots para realizar tareas específicas.
3. Mejorar el rendimiento y la adaptabilidad de los robots en entornos dinámicos y complejos.
4. Fomentar el aprendizaje y la investigación en enjambres robóticos y su aplicabilidad en diversas áreas.
5. Crear una plataforma modular y extensible para facilitar la integración de diferentes robots y sensores.

## Propósito

El propósito de este proyecto es construir y mejorar un sistema de enjambre robótico que permita a los robots trabajar de manera colaborativa para llevar a cabo tareas complejas y desafiantes en diversos entornos. Al desarrollar algoritmos y técnicas de coordinación eficientes, este proyecto busca abordar los desafíos asociados con la colaboración robótica, como la comunicación, la asignación de tareas y la adaptación a cambios en el entorno. Además, este proyecto tiene como objetivo fomentar el aprendizaje y la investigación en enjambres robóticos y su aplicabilidad en áreas como la búsqueda y rescate, la exploración, la agricultura y la logística.

## Metas de largo plazo

La versión actual cubre el control web, la coordinación escalable de TurtleBot3
Burger y la simulación ROS/Gazebo descritos en la documentación. El aprendizaje
automático y la compatibilidad con otras plataformas permanecen como líneas
futuras; no se presentan como funciones ya implementadas.

1. Implementar algoritmos de coordinación y control de enjambres.
2. Desarrollar una interfaz de comunicación eficiente y segura para facilitar la transmisión de datos entre los robots del enjambre y la estación de control central.
3. Crear una biblioteca de módulos de software reutilizables y extensibles para la asignación de tareas, navegación y control de los robots.
4. Integrar técnicas de aprendizaje y redes neuronales para mejorar la adaptabilidad y el rendimiento del enjambre robótico en entornos cambiantes y desconocidos.
5. Usar herramientas ya desarrolladas de simulación para modelar y analizar el comportamiento del enjambre robótico en diferentes entornos y escenarios.
6. Garantizar la compatibilidad con múltiples plataformas de hardware y sistemas operativos, permitiendo así la integración de una amplia variedad de robots y sensores.

## Configuración del proyecto
### Base de datos

```bash
docker compose up -d
```
### Comandos

```bash
dotnet tool install --global dotnet-ef --version 7.0.5

dotnet ef database update \
  --project SwarmBackend/SwarmBackend.csproj \
  --startup-project SwarmBackend/SwarmBackend.csproj
```

### Configuración de la base de datos

La base de datos se configura mediante el uso de Entity Framework Core.
