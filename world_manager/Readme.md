# World management package
Store environment information into POSTGIS database enabling spacial request.

## 1. Requirments
## 1.1 Install a postgis database
### 1.1.1 On OS
- from biary
see ref (https://trac.osgeo.org/postgis/wiki/UsersWikiPostGIS24UbuntuPGSQL10Apt)[https://trac.osgeo.org/postgis/wiki/UsersWikiPostGIS24UbuntuPGSQL10Apt]
- from source
see ref: [postgis documentation](https://postgis.net/docs/postgis_installation.html#install_short_version)
### 1.1.1 From Docker
- see (https://hub.docker.com/r/postgis/postgis)[https://hub.docker.com/r/postgis/postgis]

```
docker run --name some-postgis -e POSTGRES_PASSWORD=mysecretpassword -d postgis/postgis
```

## 1.2a Install pgadmin --> CAUTION LEAD TO CRASH ROS (q5 dependancies prefert docker version)



- see official doc (https://www.pgadmin.org/download/pgadmin-4-apt/)[https://www.pgadmin.org/download/pgadmin-4-apt/]
```
curl https://www.pgadmin.org/static/packages_pgadmin_org.pub | sudo apt-key add
sudo sh -c 'echo "deb https://ftp.postgresql.org/pub/pgadmin/pgadmin4/apt/$(lsb_release -cs) pgadmin4 main" > /etc/apt/sources.list.d/pgadmin4.list && apt update'

sudo apt install pgadmin4
```
## 1.2b pgadmin docker

```
docker run -p 80:80 -e 'PGADMIN_DEFAULT_EMAIL=user@domain.com' -e 'PGADMIN_DEFAULT_PASSWORD=SuperSecret' -d dpage/pgadmin4
```
- got to localhost:80 into a web browser

## 1.3 Create database
- From Pgadmin connect to the local postgis database
- Get the container IP"address running postgis 
```
docker inspect <CONTAINER_ID>  | grep IPAddress.
```
- Create a new server on pgadmin with the given IP address

Create the spatial database `world_mng_db`
- Follow instruction here (https://postgis.net/workshops/postgis-intro/creating_db.html)[https://postgis.net/workshops/postgis-intro/creating_db.html]
- do note forget to add raster extension to the DB
```
CREATE EXTENSION postgis_raster;
```


# 1.3 Installation of QGIS
follown the ref (https://www.qgis.org/fr/site/forusers/alldownloads.html)[https://www.qgis.org/fr/site/forusers/alldownloads.html]
