-- FABOS - relation machine/formation requise
-- SQL controle et idempotent: creation de la table de jointure et seed demo sans doublon.

CREATE TABLE IF NOT EXISTS MACHINE_FORMATION (
    id INT AUTO_INCREMENT PRIMARY KEY,
    machineId INT NOT NULL,
    formationId INT NOT NULL,
    requiredForAccess TINYINT(1) NOT NULL DEFAULT 1,
    niveauRequis INT NULL,
    createdAt DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_machine_formation_machine
        FOREIGN KEY (machineId)
        REFERENCES MACHINE(id)
        ON DELETE CASCADE,
    CONSTRAINT fk_machine_formation_formation
        FOREIGN KEY (formationId)
        REFERENCES FORMATION(id)
        ON DELETE CASCADE,
    CONSTRAINT unique_machine_formation
        UNIQUE (machineId, formationId)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

INSERT INTO MACHINE_FORMATION (machineId, formationId, requiredForAccess, niveauRequis)
SELECT m.id, f.id, 1, 1
FROM MACHINE m
JOIN FORMATION f ON f.titre = 'Formation imprimante 3D'
WHERE m.machineToken IN ('printer-01', 'ultimaker-s5-01', 'prusa-mk3s-01')
  AND NOT EXISTS (
      SELECT 1
      FROM MACHINE_FORMATION mf
      WHERE mf.machineId = m.id
        AND mf.formationId = f.id
  );
