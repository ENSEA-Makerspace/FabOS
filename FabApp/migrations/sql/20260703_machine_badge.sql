-- FABOS - MACHINE_BADGE et migration controlee depuis MACHINE_FORMATION
CREATE TABLE IF NOT EXISTS MACHINE_BADGE (
    id INT AUTO_INCREMENT PRIMARY KEY,
    machineId INT NOT NULL,
    badgeId INT NOT NULL,
    requiredForAccess TINYINT(1) NOT NULL DEFAULT 1,
    createdAt DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    UNIQUE KEY unique_machine_badge (machineId, badgeId),
    CONSTRAINT fk_machine_badge_machine FOREIGN KEY (machineId) REFERENCES MACHINE(id) ON DELETE CASCADE,
    CONSTRAINT fk_machine_badge_badge FOREIGN KEY (badgeId) REFERENCES BADGE(id) ON DELETE CASCADE
);

INSERT INTO MACHINE_BADGE (machineId, badgeId, requiredForAccess)
SELECT DISTINCT mf.machineId, f.badgeId, 1
FROM MACHINE_FORMATION mf
INNER JOIN FORMATION f ON f.id = mf.formationId
WHERE mf.requiredForAccess = 1
  AND f.badgeId IS NOT NULL
  AND NOT EXISTS (
      SELECT 1 FROM MACHINE_BADGE mb
      WHERE mb.machineId = mf.machineId AND mb.badgeId = f.badgeId
  );

INSERT INTO UTILISATEUR_BADGE (utilisateurId, badgeId)
SELECT DISTINCT p.userId, f.badgeId
FROM PROGRESSION p
INNER JOIN FORMATION f ON f.id = p.formationId
WHERE p.completed = 1
  AND f.badgeId IS NOT NULL
  AND NOT EXISTS (
      SELECT 1 FROM UTILISATEUR_BADGE ub
      WHERE ub.utilisateurId = p.userId AND ub.badgeId = f.badgeId
  );
