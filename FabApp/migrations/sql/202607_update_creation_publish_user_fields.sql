-- Migration manuelle FABOS - compléments Creation pour publication utilisateur.
-- À exécuter dans Adminer sur la base fabos, sans doctrine:schema:update --force.

ALTER TABLE CREATION
    ADD COLUMN IF NOT EXISTS printDurationMinutes INT DEFAULT NULL AFTER externalUrl;

ALTER TABLE CREATION
    MODIFY isPublished TINYINT(1) NOT NULL DEFAULT 1;

CREATE INDEX IF NOT EXISTS IDX_CREATION_PUBLISHED_CREATED ON CREATION (isPublished, createdAt);
