-- Table: public.Object

-- DROP TABLE public."Object";


-- SCHEMA: public

-- DROP SCHEMA public ;

--CREATE SCHEMA public
--    AUTHORIZATION postgres;
--
--COMMENT ON SCHEMA public
--    IS 'standard public schema';
--
--GRANT ALL ON SCHEMA public TO PUBLIC;
--
--GRANT ALL ON SCHEMA public TO postgres;


DROP TABLE IF EXISTS public."object";

CREATE TABLE public."object"
(
    id character varying COLLATE pg_catalog."default" NOT NULL,
    coordinate geometry NOT NULL,
    type character varying COLLATE pg_catalog."default" NOT NULL,
    type_name character varying COLLATE pg_catalog."default",
    orient_x double precision,
    orient_y double precision,
    orient_z double precision,
    orient_w double precision,
    ttl real NOT NULL,
    confidence real,
    CONSTRAINT "object_pkey" PRIMARY KEY (id)
);


SET standard_conforming_strings = OFF;
SET postgis.gdal_enabled_drivers = 'ENABLE_ALL';
DROP TABLE IF EXISTS public."room";
-- DELETE FROM geometry_columns WHERE f_table_name = 'room' AND f_table_schema = 'public';
-- BEGIN;


CREATE TABLE public."room" ( "ogc_fid" SERIAL, CONSTRAINT "room_pk" PRIMARY KEY ("ogc_fid") );
SELECT AddGeometryColumn('public','room','poly',4326,'POLYGON',2);
CREATE INDEX "room_Room_geom_idx" ON "public"."room" USING GIST ("poly");

ALTER TABLE public."room" ADD COLUMN "room" VARCHAR(10);
INSERT INTO public."room" ("poly" , "room") VALUES ('0103000020E61000000100000007000000DA39B8118C6CFA3F3041829E8E9B0F4026FD73732A5EF63F3041829E8E9B0F404274EF7B07D4F43F357A59F8A8BFF03F1089A7434DDA1440E00327D69350EE3F7C1EACD36105154084C89C4F2E080F407C1EACD36105154084C89C4F2E080F40DA39B8118C6CFA3F3041829E8E9B0F40', 'Kitchen');
INSERT INTO public."room" ("poly" , "room") VALUES ('0103000020E61000000100000006000000C513C88546F1F43F289A860DBCBAED3FBDB52EA28102F43FC8C60D098D4BF3BF7094D44C02A01440D024A7EC513AF4BF02219E5E3DD414401C8D20B89454EC3F02219E5E3DD414401C8D20B89454EC3FC513C88546F1F43F289A860DBCBAED3F', 'Bedroom');
INSERT INTO public."room" ("poly" , "room") VALUES ('0103000020E61000000100000006000000920BB64F3A15FBBF9CB15F37E7321040181269FA4DC8FBBF661416BFE6141D4048B4315FB0061640C1A39C62BF701C40C2AD7EB49C531540AEAD3F52583B0F40C2AD7EB49C531540AEAD3F52583B0F40920BB64F3A15FBBF9CB15F37E7321040', 'LivingRoom');

DROP TABLE IF EXISTS public."map_raster";
CREATE TABLE map_raster(rid serial primary key, map raster);

--TABLESPACE pg_default;

ALTER TABLE public."object"
    OWNER to postgres;

ALTER TABLE public."room"
    OWNER to postgres;

ALTER TABLE public."map_raster"
    OWNER to postgres;