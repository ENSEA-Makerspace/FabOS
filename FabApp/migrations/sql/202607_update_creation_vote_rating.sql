-- Migration manuelle FABOS - notation par étoiles des créations.
-- À exécuter dans Adminer sur la base FABOS, sans doctrine:schema:update --force.
-- Les anciens votes simples sont conservés et reçoivent rating = 5.0.

SET @schema_name = DATABASE();

SET @sql = IF(
    (SELECT COUNT(*) FROM INFORMATION_SCHEMA.COLUMNS WHERE TABLE_SCHEMA = @schema_name AND TABLE_NAME = 'CREATION_VOTE' AND COLUMN_NAME = 'rating') = 0,
    'ALTER TABLE CREATION_VOTE ADD rating FLOAT NOT NULL DEFAULT 5.0 AFTER userId',
    'SELECT ''CREATION_VOTE.rating existe déjà'' AS info'
);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;

SET @sql = IF(
    (SELECT COUNT(*) FROM INFORMATION_SCHEMA.COLUMNS WHERE TABLE_SCHEMA = @schema_name AND TABLE_NAME = 'CREATION_VOTE' AND COLUMN_NAME = 'updatedAt') = 0,
    'ALTER TABLE CREATION_VOTE ADD updatedAt DATETIME DEFAULT NULL AFTER createdAt',
    'SELECT ''CREATION_VOTE.updatedAt existe déjà'' AS info'
);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;

UPDATE CREATION_VOTE
SET rating = 5.0
WHERE rating IS NULL OR rating < 0.5 OR rating > 5.0;

SET @unique_exists = (
    SELECT COUNT(*) FROM (
        SELECT INDEX_NAME
        FROM INFORMATION_SCHEMA.STATISTICS
        WHERE TABLE_SCHEMA = @schema_name
          AND TABLE_NAME = 'CREATION_VOTE'
          AND NON_UNIQUE = 0
        GROUP BY INDEX_NAME
        HAVING GROUP_CONCAT(COLUMN_NAME ORDER BY SEQ_IN_INDEX SEPARATOR ',') = 'creationId,userId'
    ) unique_indexes
);

SET @sql = IF(
    @unique_exists = 0,
    'ALTER TABLE CREATION_VOTE ADD UNIQUE INDEX UNIQ_CREATION_VOTE_USER (creationId, userId)',
    'SELECT ''Contrainte unique creationId/userId déjà présente'' AS info'
);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;
