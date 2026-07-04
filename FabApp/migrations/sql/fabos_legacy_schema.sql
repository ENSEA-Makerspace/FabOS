-- ============================================================
-- FABOS - Base MariaDB corrigée
-- Compatible MariaDB / MySQL
-- ============================================================

SET FOREIGN_KEY_CHECKS = 0;

DROP TABLE IF EXISTS CHOIX;
DROP TABLE IF EXISTS QUESTION;
DROP TABLE IF EXISTS QUIZ;
DROP TABLE IF EXISTS SECTION;
DROP TABLE IF EXISTS PROGRESSION;
DROP TABLE IF EXISTS LOG_UTILISATION;
DROP TABLE IF EXISTS ACCESS_RFID_LOG;
DROP TABLE IF EXISTS RESERVATION;
DROP TABLE IF EXISTS MACHINE_FORMATION;
DROP TABLE IF EXISTS MACHINE_UTILISATEUR;
DROP TABLE IF EXISTS UTILISATEUR_BADGE;
DROP TABLE IF EXISTS UTILISATEUR_ROLE;
DROP TABLE IF EXISTS FORMATION;
DROP TABLE IF EXISTS MACHINE;
DROP TABLE IF EXISTS UTILISATEUR;
DROP TABLE IF EXISTS ROLE;
DROP TABLE IF EXISTS BADGE;

SET FOREIGN_KEY_CHECKS = 1;

-- ============================================================
-- TABLES DE RÉFÉRENCE
-- ============================================================

CREATE TABLE BADGE (
    id INT AUTO_INCREMENT PRIMARY KEY,
    nom VARCHAR(255) NOT NULL,
    description TEXT,
    icone VARCHAR(255),
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE ROLE (
    id INT AUTO_INCREMENT PRIMARY KEY,
    nom VARCHAR(50) NOT NULL UNIQUE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ============================================================
-- TABLES PRINCIPALES
-- ============================================================

CREATE TABLE UTILISATEUR (
    id INT AUTO_INCREMENT PRIMARY KEY,
    email VARCHAR(255) NOT NULL UNIQUE,
    username VARCHAR(255) NOT NULL UNIQUE,
    password VARCHAR(255) NOT NULL,
    firstName VARCHAR(255),
    lastName VARCHAR(255),
    statut VARCHAR(50) DEFAULT 'actif',
    identifiantRfid VARCHAR(255) UNIQUE,
    tempsPresenceTotal INT DEFAULT 0,
    isVerified TINYINT(1) DEFAULT 0,
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE MACHINE (
    id INT AUTO_INCREMENT PRIMARY KEY,
    nom VARCHAR(255) NOT NULL,
    description TEXT,
    localisation VARCHAR(255),
    photo VARCHAR(255),
    statut VARCHAR(50) NOT NULL DEFAULT 'idle',
    granularite VARCHAR(50),
    categorySlug VARCHAR(100),
    categoryLabel VARCHAR(100),
    levelSlug VARCHAR(50),
    levelLabel VARCHAR(50),
    iconSlug VARCHAR(50),
    materials JSON,
    features JSON,
    requirementTitle VARCHAR(255),
    requirementDescription TEXT,
    popularity INT,
    limiteReservations INT DEFAULT 0,
    machineToken VARCHAR(255) UNIQUE NOT NULL,
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    lastAuthorizationTime TIMESTAMP NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE FORMATION (
    id INT AUTO_INCREMENT PRIMARY KEY,
    badgeId INT NULL,
    titre VARCHAR(255) NOT NULL,
    description TEXT,
    image VARCHAR(255),
    categorie VARCHAR(100),
    niveau INT,
    duree VARCHAR(100),
    formateur VARCHAR(150),
    placesTotales INT,
    objectifs TEXT,
    prerequis TEXT,
    materielFourni TEXT,

    CONSTRAINT fk_formation_badge
        FOREIGN KEY (badgeId)
        REFERENCES BADGE(id)
        ON DELETE SET NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ============================================================
-- TABLES DE JONCTION
-- ============================================================

CREATE TABLE UTILISATEUR_ROLE (
    utilisateurId INT NOT NULL,
    roleId INT NOT NULL,

    PRIMARY KEY (utilisateurId, roleId),

    CONSTRAINT fk_utilisateur_role_user
        FOREIGN KEY (utilisateurId)
        REFERENCES UTILISATEUR(id)
        ON DELETE CASCADE,

    CONSTRAINT fk_utilisateur_role_role
        FOREIGN KEY (roleId)
        REFERENCES ROLE(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE UTILISATEUR_BADGE (
    utilisateurId INT NOT NULL,
    badgeId INT NOT NULL,
    dateObtention TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    PRIMARY KEY (utilisateurId, badgeId),

    CONSTRAINT fk_utilisateur_badge_user
        FOREIGN KEY (utilisateurId)
        REFERENCES UTILISATEUR(id)
        ON DELETE CASCADE,

    CONSTRAINT fk_utilisateur_badge_badge
        FOREIGN KEY (badgeId)
        REFERENCES BADGE(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE MACHINE_UTILISATEUR (
    machineId INT NOT NULL,
    userId INT NOT NULL,
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    PRIMARY KEY (machineId, userId),

    CONSTRAINT fk_machine_utilisateur_machine
        FOREIGN KEY (machineId)
        REFERENCES MACHINE(id)
        ON DELETE CASCADE,

    CONSTRAINT fk_machine_utilisateur_user
        FOREIGN KEY (userId)
        REFERENCES UTILISATEUR(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;


CREATE TABLE MACHINE_FORMATION (
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

-- ============================================================
-- RÉSERVATIONS
-- ============================================================

CREATE TABLE RESERVATION (
    id INT AUTO_INCREMENT PRIMARY KEY,
    userId INT NOT NULL,
    machineId INT NOT NULL,
    dateDebut DATETIME NOT NULL,
    dateFin DATETIME NOT NULL,
    motif TEXT,
    created TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    CONSTRAINT fk_reservation_user
        FOREIGN KEY (userId)
        REFERENCES UTILISATEUR(id)
        ON DELETE CASCADE,

    CONSTRAINT fk_reservation_machine
        FOREIGN KEY (machineId)
        REFERENCES MACHINE(id)
        ON DELETE CASCADE,

    CONSTRAINT chk_reservation_dates
        CHECK (dateFin > dateDebut)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ============================================================
-- LOGS D'UTILISATION MACHINE
-- ============================================================

CREATE TABLE LOG_UTILISATION (
    id INT AUTO_INCREMENT PRIMARY KEY,
    machineId INT NOT NULL,
    userId INT NOT NULL,
    dateDebut DATETIME NOT NULL,
    dateFin DATETIME NULL,
    duree INT NULL,
    source VARCHAR(50),
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    CONSTRAINT fk_log_machine
        FOREIGN KEY (machineId)
        REFERENCES MACHINE(id)
        ON DELETE CASCADE,

    CONSTRAINT fk_log_user
        FOREIGN KEY (userId)
        REFERENCES UTILISATEUR(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ============================================================
-- LOGS RFID / ACCÈS
-- ============================================================
-- Table utile pour ton test RFID.
-- userId peut être NULL pour garder les logs de badges inconnus.

CREATE TABLE ACCESS_RFID_LOG (
    id INT AUTO_INCREMENT PRIMARY KEY,
    badgeUid VARCHAR(255) NOT NULL,
    userId INT NULL,
    machineId INT NULL,
    authorized TINYINT(1) NOT NULL DEFAULT 0,
    status VARCHAR(100) NOT NULL,
    reason VARCHAR(255),
    message TEXT,
    color VARCHAR(50),
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    CONSTRAINT fk_access_rfid_user
        FOREIGN KEY (userId)
        REFERENCES UTILISATEUR(id)
        ON DELETE SET NULL,

    CONSTRAINT fk_access_rfid_machine
        FOREIGN KEY (machineId)
        REFERENCES MACHINE(id)
        ON DELETE SET NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ============================================================
-- FORMATIONS / PROGRESSION
-- ============================================================

CREATE TABLE PROGRESSION (
    id INT AUTO_INCREMENT PRIMARY KEY,
    userId INT NOT NULL,
    formationId INT NOT NULL,
    score INT DEFAULT 0,
    completed TINYINT(1) DEFAULT 0,
    dateDebut TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    dateEnd DATETIME NULL,

    CONSTRAINT fk_progression_user
        FOREIGN KEY (userId)
        REFERENCES UTILISATEUR(id)
        ON DELETE CASCADE,

    CONSTRAINT fk_progression_formation
        FOREIGN KEY (formationId)
        REFERENCES FORMATION(id)
        ON DELETE CASCADE,

    CONSTRAINT chk_progression_dates
        CHECK (dateEnd IS NULL OR dateEnd > dateDebut),

    CONSTRAINT unique_user_formation
        UNIQUE (userId, formationId)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE SECTION (
    id INT AUTO_INCREMENT PRIMARY KEY,
    formationId INT NOT NULL,
    titre VARCHAR(255) NOT NULL,
    contenu TEXT,
    videoUrl VARCHAR(255),
    ordre INT NOT NULL,
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    CONSTRAINT fk_section_formation
        FOREIGN KEY (formationId)
        REFERENCES FORMATION(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE QUIZ (
    id INT AUTO_INCREMENT PRIMARY KEY,
    formationId INT NOT NULL,
    sectionId INT NULL,
    noteMinimale INT DEFAULT 0,

    CONSTRAINT fk_quiz_formation
        FOREIGN KEY (formationId)
        REFERENCES FORMATION(id)
        ON DELETE CASCADE,

    CONSTRAINT fk_quiz_section
        FOREIGN KEY (sectionId)
        REFERENCES SECTION(id)
        ON DELETE SET NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE QUESTION (
    id INT AUTO_INCREMENT PRIMARY KEY,
    quizId INT NOT NULL,
    texte TEXT NOT NULL,
    type VARCHAR(50) NOT NULL,
    ordre INT NOT NULL,

    CONSTRAINT fk_question_quiz
        FOREIGN KEY (quizId)
        REFERENCES QUIZ(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE CHOIX (
    id INT AUTO_INCREMENT PRIMARY KEY,
    questionId INT NOT NULL,
    texte TEXT NOT NULL,
    estCorrect TINYINT(1) NOT NULL,
    ordre INT NOT NULL,

    CONSTRAINT fk_choix_question
        FOREIGN KEY (questionId)
        REFERENCES QUESTION(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ============================================================
-- INDEX POUR PERFORMANCE
-- ============================================================

CREATE INDEX idx_utilisateur_rfid
    ON UTILISATEUR(identifiantRfid);

CREATE INDEX idx_machine_token
    ON MACHINE(machineToken);

CREATE INDEX idx_reservation_user
    ON RESERVATION(userId);

CREATE INDEX idx_reservation_machine
    ON RESERVATION(machineId);

CREATE INDEX idx_reservation_dates
    ON RESERVATION(dateDebut, dateFin);

CREATE INDEX idx_log_utilisation_machine
    ON LOG_UTILISATION(machineId);

CREATE INDEX idx_log_utilisation_user
    ON LOG_UTILISATION(userId);

CREATE INDEX idx_access_rfid_badge
    ON ACCESS_RFID_LOG(badgeUid);

CREATE INDEX idx_access_rfid_user
    ON ACCESS_RFID_LOG(userId);

CREATE INDEX idx_access_rfid_machine
    ON ACCESS_RFID_LOG(machineId);

CREATE INDEX idx_access_rfid_created
    ON ACCESS_RFID_LOG(createdAt);

CREATE INDEX idx_progression_user
    ON PROGRESSION(userId);

CREATE INDEX idx_progression_formation
    ON PROGRESSION(formationId);

CREATE INDEX idx_section_formation
    ON SECTION(formationId);

CREATE INDEX idx_quiz_formation
    ON QUIZ(formationId);

CREATE INDEX idx_question_quiz
    ON QUESTION(quizId);

CREATE INDEX idx_choix_question
    ON CHOIX(questionId);

-- ============================================================
-- DONNÉES DE TEST
-- ============================================================

INSERT INTO ROLE (nom) VALUES
('admin'),
('staff'),
('user');

INSERT INTO BADGE (nom, description, icone) VALUES
('Maker 3D', 'Badge obtenu après formation imprimante 3D', 'printer-3d');

INSERT INTO MACHINE (
    nom,
    description,
    localisation,
    statut,
    granularite,
    categorySlug,
    categoryLabel,
    levelSlug,
    levelLabel,
    iconSlug,
    materials,
    features,
    requirementTitle,
    requirementDescription,
    popularity,
    limiteReservations,
    machineToken
) VALUES
(
    'Imprimante 3D test',
    'Machine de test pour FABOS',
    'Zone gauche',
    'idle',
    '30min',
    'impression-3d',
    'Impression 3D',
    'niveau-1',
    'Niveau 1',
    'impression-3d',
    JSON_ARRAY('PLA', 'PETG', 'TPU', 'Support'),
    JSON_ARRAY('Volume utile standard FabLab', 'Réservation par créneau', 'Utilisation accompagnée possible', 'Traçabilité RFID'),
    'Formation imprimante 3D recommandée',
    'Validation formation ou accompagnement staff conseillé avant première utilisation.',
    4,
    2,
    'printer-01'
);

INSERT INTO FORMATION (
    badgeId,
    titre,
    description,
    image,
    categorie,
    niveau,
    duree,
    formateur,
    placesTotales,
    objectifs,
    prerequis,
    materielFourni
) VALUES
(
    1,
    'Formation imprimante 3D',
    'Formation obligatoire pour utiliser les imprimantes 3D.',
    NULL,
    'Impression 3D',
    1,
    '2h',
    'Équipe FabLab',
    8,
    'Comprendre les règles de sécurité liées à l’impression 3D. Préparer un modèle dans le slicer. Choisir les paramètres de base pour le PLA. Lancer, surveiller et retirer une impression simple.',
    'Aucun prérequis technique obligatoire. Bases de sécurité atelier et compte FabOS actif recommandés.',
    'Imprimante 3D, filament PLA, outils de calibration, fiche sécurité.'
);


INSERT INTO MACHINE_FORMATION (
    machineId,
    formationId,
    requiredForAccess,
    niveauRequis
) VALUES
(
    1,
    1,
    1,
    1
);

INSERT INTO UTILISATEUR (
    email,
    username,
    password,
    firstName,
    lastName,
    statut,
    identifiantRfid,
    isVerified
) VALUES
(
    'yanis@example.com',
    'yanis',
    'password_hash_test',
    'Yanis',
    'Test',
    'actif',
    '8C52B359',
    1
),
(
    'sofia@example.com',
    'sofia',
    'password_hash_test',
    'Sofia',
    'Test',
    'actif',
    '39F1C387',
    1
);

-- Yanis est formé.
INSERT INTO PROGRESSION (
    userId,
    formationId,
    score,
    completed,
    dateEnd
) VALUES
(
    1,
    1,
    100,
    1,
    CURRENT_TIMESTAMP
);

-- Sofia a commencé mais n'a pas validé.
INSERT INTO PROGRESSION (
    userId,
    formationId,
    score,
    completed
) VALUES
(
    2,
    1,
    40,
    0
);

-- Autorisation explicite machine-utilisateur pour Yanis.
INSERT INTO MACHINE_UTILISATEUR (
    machineId,
    userId
) VALUES
(
    1,
    1
);
