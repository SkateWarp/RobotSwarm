FROM mcr.microsoft.com/dotnet/sdk:8.0 AS build

WORKDIR /src
COPY SwarmBackend/SwarmBackend.csproj SwarmBackend/
RUN dotnet restore SwarmBackend/SwarmBackend.csproj

COPY SwarmBackend/ SwarmBackend/
WORKDIR /src/SwarmBackend
RUN dotnet publish SwarmBackend.csproj \
    --configuration Release \
    --no-restore \
    --output /app

FROM mcr.microsoft.com/dotnet/aspnet:8.0 AS runtime

USER root
RUN apt-get update \
    && apt-get install --yes --no-install-recommends curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=build /app .

ENV ASPNETCORE_ENVIRONMENT=Production
EXPOSE 44336

USER app
ENTRYPOINT ["dotnet", "SwarmBackend.dll"]
