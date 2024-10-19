FROM mcr.microsoft.com/dotnet/sdk:8.0 AS build
WORKDIR /src
COPY ["SwarmBackend/SwarmBackend.csproj", "SwarmBackend/"]
RUN dotnet restore "SwarmBackend/SwarmBackend.csproj"
COPY . .
WORKDIR "/src/SwarmBackend"
RUN dotnet build "SwarmBackend.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish "SwarmBackend.csproj" -c Release -o /app/publish

FROM mcr.microsoft.com/dotnet/aspnet:8.0 AS final
WORKDIR /app
COPY --from=publish /app/publish .
COPY --from=build /src /src
RUN apt-get update && apt-get install -y postgresql-client

# Install EF Core tools
RUN dotnet tool install --global dotnet-ef
ENV PATH="$PATH:/root/.dotnet/tools"

# Create startup script
RUN echo '#!/bin/bash\n\
    dotnet ef database update --project /src/SwarmBackend\n\
    dotnet SwarmBackend.dll' > /app/startup.sh && chmod +x /app/startup.sh

ENTRYPOINT ["/app/startup.sh"]