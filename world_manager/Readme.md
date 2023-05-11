# World management package
Store environment information into POSTGIS database enabling spacial request.

## 1. Requirments
## 1.1 Install a postgis database

### 1.1.1 From Docker
- see [https://hub.docker.com/r/postgis/postgis](https://hub.docker.com/r/postgis/postgis)

```
docker run --name postgis-world-mng --restart unless-stopped -e POSTGRES_PASSWORD=mysecretpassword -e APP_DB_NAME=world_mng_db -v <local path>/db:/docker-entrypoint-initdb.d/ -d postgis/postgis
```
- Here the **01-init.sh** file is executed at the docker start, create the db *world_mng_db* and adding postgis extension **postgis**, **postgis_raster**
- A default postgres user is created with the associated pwd (in docker run command line)

# 2 Installation of QGIS
follown the ref [https://www.qgis.org/fr/site/forusers/alldownloads.html](https://www.qgis.org/fr/site/forusers/alldownloads.html)


# 3 Create geottif from Ros map
- Start a ros core
```
roscore
```
- publish current map
```
rosrun map_server map_server map.yaml
```

- got to the tools repository
```
 roscd world_manager/scripts/process/tools/
```

- transform map topic content to map.json
```
python map_to_json.py
```
- Close the programm when the following message appear `map.json file successfully created, please close this node`

- Convert the json map to geotiff, execute the following command
```
python3 msg_to_geotiff.py  ./map.json '../../tmp/map.gtiff'
```
- the `map.gtiff` is created and can be added to QGIS



## 4 Optional
## 4 pgadmin docker

```
docker run -p 80:80 -e 'PGADMIN_DEFAULT_EMAIL=user@domain.com' -e 'PGADMIN_DEFAULT_PASSWORD=SuperSecret' -d dpage/pgadmin4
```
- got to localhost:80 into a web browser