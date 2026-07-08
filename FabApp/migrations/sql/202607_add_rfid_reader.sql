-- FABOS - lecteurs RFID physiques Raspberry Pi
-- Script controle: a executer manuellement apres validation.

CREATE TABLE IF NOT EXISTS RFID_READER (
    id INT AUTO_INCREMENT PRIMARY KEY,
    name VARCHAR(120) NOT NULL,
    readerToken VARCHAR(120) NOT NULL UNIQUE,
    machineId INT NOT NULL,
    isActive TINYINT(1) NOT NULL DEFAULT 1,
    lastSeenAt DATETIME DEFAULT NULL,
    createdAt DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updatedAt DATETIME DEFAULT NULL,

    INDEX idx_rfid_reader_machine (machineId),

    CONSTRAINT fk_rfid_reader_machine
        FOREIGN KEY (machineId)
        REFERENCES MACHINE(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;
