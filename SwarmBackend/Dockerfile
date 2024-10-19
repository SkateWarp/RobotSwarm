FROM mcr.microsoft.com/dotnet/aspnet:8.0 AS base
WORKDIR /app
EXPOSE 80

FROM mcr.microsoft.com/dotnet/sdk:8.0 AS build
WORKDIR /src
COPY ["SwarmBackend/SwarmBackend.csproj", "SwarmBackend/"]
RUN dotnet restore "SwarmBackend/SwarmBackend.csproj"
COPY . .
WORKDIR "/src/SwarmBackend"
RUN dotnet build "SwarmBackend.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish "SwarmBackend.csproj" -c Release -o /app/publish

FROM base AS final
WORKDIR /app
COPY --from=publish /app/publish .
ENTRYPOINT ["dotnet", "SwarmBackend.dll"]