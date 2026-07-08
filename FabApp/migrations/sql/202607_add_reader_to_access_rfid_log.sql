-- Relie les logs RFID au lecteur RFID déclaré dans l'admin.
-- A exécuter manuellement dans Adminer sur la base MariaDB fabos.
-- Les anciens logs restent conservés. readerId est nullable et la FK utilise ON DELETE SET NULL.

SET @schema_name = DATABASE();

SET @sql = IF(
    EXISTS (
        SELECT 1
        FROM information_schema.COLUMNS
        WHERE TABLE_SCHEMA = @schema_name
          AND TABLE_NAME = 'ACCESS_RFID_LOG'
          AND COLUMN_NAME = 'readerId'
    ),
    'SELECT ''ACCESS_RFID_LOG.readerId existe déjà''',
    'ALTER TABLE ACCESS_RFID_LOG ADD COLUMN readerId INT DEFAULT NULL'
);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;

SET @sql = IF(
    EXISTS (
        SELECT 1
        FROM information_schema.COLUMNS
        WHERE TABLE_SCHEMA = @schema_name
          AND TABLE_NAME = 'ACCESS_RFID_LOG'
          AND COLUMN_NAME = 'readerToken'
    ),
    'SELECT ''ACCESS_RFID_LOG.readerToken existe déjà''',
    'ALTER TABLE ACCESS_RFID_LOG ADD COLUMN readerToken VARCHAR(120) DEFAULT NULL'
);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;

SET @sql = IF(
    EXISTS (
        SELECT 1
        FROM information_schema.STATISTICS
        WHERE TABLE_SCHEMA = @schema_name
          AND TABLE_NAME = 'ACCESS_RFID_LOG'
          AND INDEX_NAME = 'idx_access_rfid_log_reader_id'
    ),
    'SELECT ''Index idx_access_rfid_log_reader_id existe déjà''',
    'ALTER TABLE ACCESS_RFID_LOG ADD INDEX idx_access_rfid_log_reader_id (readerId)'
);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;

SET @sql = IF(
    EXISTS (
        SELECT 1
        FROM information_schema.STATISTICS
        WHERE TABLE_SCHEMA = @schema_name
          AND TABLE_NAME = 'ACCESS_RFID_LOG'
          AND INDEX_NAME = 'idx_access_rfid_log_reader_token'
    ),
    'SELECT ''Index idx_access_rfid_log_reader_token existe déjà''',
    'ALTER TABLE ACCESS_RFID_LOG ADD INDEX idx_access_rfid_log_reader_token (readerToken)'
);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;

SET @sql = IF(
    EXISTS (
        SELECT 1
        FROM information_schema.TABLE_CONSTRAINTS
        WHERE CONSTRAINT_SCHEMA = @schema_name
          AND TABLE_NAME = 'ACCESS_RFID_LOG'
          AND CONSTRAINT_NAME = 'fk_access_rfid_log_reader'
          AND CONSTRAINT_TYPE = 'FOREIGN KEY'
    ),
    'SELECT ''FK fk_access_rfid_log_reader existe déjà''',
    'ALTER TABLE ACCESS_RFID_LOG ADD CONSTRAINT fk_access_rfid_log_reader FOREIGN KEY (readerId) REFERENCES RFID_READER(id) ON DELETE SET NULL'
);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;
