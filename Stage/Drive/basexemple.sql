-- ========== TABLES DE RÉFÉRENCE (sans dépendances) ==========

-- Badges disponibles dans le système
CREATE TABLE BADGE (
    id SERIAL PRIMARY KEY,
    nom VARCHAR(255) NOT NULL,
    description TEXT,
    icone VARCHAR(255),
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Machines disponibles
CREATE TABLE MACHINE (
    id SERIAL PRIMARY KEY,
    nom VARCHAR(255) NOT NULL,
    description TEXT,
    localisation VARCHAR(255),
    photo VARCHAR(255),
    statut VARCHAR(50) NOT NULL,
    granularite VARCHAR(50),
    limiteReservations INTEGER DEFAULT 0,
    machineToken VARCHAR(255) UNIQUE NOT NULL,
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    lastAuthorizationTime TIMESTAMP
);

-- Formations disponibles
CREATE TABLE FORMATION (
    id SERIAL PRIMARY KEY,
    badgeId INTEGER,
    titre VARCHAR(255) NOT NULL,
    desc TEXT,
    image VARCHAR(255),
    FOREIGN KEY (badgeId) REFERENCES BADGE(id) ON DELETE SET NULL
);

-- ========== TABLES DE JONCTION POUR LES TABLEAUX ==========

-- Rôles des utilisateurs (pour gérer string[] roles)
CREATE TABLE ROLE (
    id SERIAL PRIMARY KEY,
    nom VARCHAR(50) NOT NULL UNIQUE
);

CREATE TABLE UTILISATEUR_ROLE (
    utilisateurId INTEGER NOT NULL,
    roleId INTEGER NOT NULL,
    PRIMARY KEY (utilisateurId, roleId),
    FOREIGN KEY (utilisateurId) REFERENCES UTILISATEUR(id) ON DELETE CASCADE,
    FOREIGN KEY (roleId) REFERENCES ROLE(id) ON DELETE CASCADE
);

CREATE TABLE MACHINE_BADGE (
    machineId INTEGER NOT NULL,
    badgeId INTEGER NOT NULL,
    PRIMARY KEY (machineId, badgeId),
    FOREIGN KEY (machineId) REFERENCES MACHINE(id) ON DELETE CASCADE,
    FOREIGN KEY (badgeId) REFERENCES BADGE(id) ON DELETE CASCADE
);


-- Badges des utilisateurs (pour gérer int[] badgeList)
CREATE TABLE UTILISATEUR_BADGE (
    utilisateurId INTEGER NOT NULL,
    badgeId INTEGER NOT NULL,
    dateObtention TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (utilisateurId, badgeId),
    FOREIGN KEY (utilisateurId) REFERENCES UTILISATEUR(id) ON DELETE CASCADE,
    FOREIGN KEY (badgeId) REFERENCES BADGE(id) ON DELETE CASCADE
);

-- ========== TABLES PRINCIPALES ==========

-- Utilisateurs
CREATE TABLE UTILISATEUR (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) NOT NULL UNIQUE,
    username VARCHAR(255) NOT NULL UNIQUE,
    password VARCHAR(255) NOT NULL,
    firstName VARCHAR(255),
    lastName VARCHAR(255),
    statut VARCHAR(50) DEFAULT 'actif',
    identifiantRfid VARCHAR(255) UNIQUE,
    tempsPresenceTotal INTEGER DEFAULT 0,
    isVerified BOOLEAN DEFAULT FALSE,
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Réservations de machines
CREATE TABLE RESERVATION (
    id SERIAL PRIMARY KEY,
    userId INTEGER NOT NULL,
    machineId INTEGER NOT NULL,
    dateDebut TIMESTAMP NOT NULL,
    dateFin TIMESTAMP NOT NULL,
    motif TEXT,
    created TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (userId) REFERENCES UTILISATEUR(id) ON DELETE CASCADE,
    FOREIGN KEY (machineId) REFERENCES MACHINE(id) ON DELETE CASCADE,
    CONSTRAINT chk_dates CHECK (dateFin > dateDebut)
);

-- Logs d'utilisation des machines
CREATE TABLE LOG_UTILISATION (
    id SERIAL PRIMARY KEY,
    machineId INTEGER NOT NULL,
    userId INTEGER NOT NULL,
    dateDebut TIMESTAMP NOT NULL,
    dateFin TIMESTAMP,
    duree INTEGER, -- en minutes
    source VARCHAR(50),
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (machineId) REFERENCES MACHINE(id) ON DELETE CASCADE,
    FOREIGN KEY (userId) REFERENCES UTILISATEUR(id) ON DELETE CASCADE
);

-- Progression des utilisateurs dans les formations
CREATE TABLE PROGRESSION (
    id SERIAL PRIMARY KEY,
    userId INTEGER NOT NULL,
    formationId INTEGER NOT NULL,
    score INTEGER DEFAULT 0,
    completed BOOLEAN DEFAULT FALSE,
    dateDebut TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    dateEnd TIMESTAMP,
    FOREIGN KEY (userId) REFERENCES UTILISATEUR(id) ON DELETE CASCADE,
    FOREIGN KEY (formationId) REFERENCES FORMATION(id) ON DELETE CASCADE,
    CONSTRAINT chk_dates_progression CHECK (dateEnd IS NULL OR dateEnd > dateDebut)
);

-- Sections des formations
CREATE TABLE SECTION (
    id SERIAL PRIMARY KEY,
    formationId INTEGER NOT NULL,
    titre VARCHAR(255) NOT NULL,
    contenu TEXT,
    videoUrl VARCHAR(255),
    ordre INTEGER NOT NULL,
    createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (formationId) REFERENCES FORMATION(id) ON DELETE CASCADE
);

-- Quiz des formations
CREATE TABLE QUIZ (
    id SERIAL PRIMARY KEY,
    formationId INTEGER NOT NULL,
    sectionId INTEGER,
    noteMinimale INTEGER DEFAULT 0,
    FOREIGN KEY (formationId) REFERENCES FORMATION(id) ON DELETE CASCADE,
    FOREIGN KEY (sectionId) REFERENCES SECTION(id) ON DELETE SET NULL
);

-- Questions des quiz
CREATE TABLE QUESTION (
    id SERIAL PRIMARY KEY,
    quizId INTEGER NOT NULL,
    texte TEXT NOT NULL,
    type VARCHAR(50) NOT NULL, -- 'qcm', 'texte', etc.
    ordre INTEGER NOT NULL,
    FOREIGN KEY (quizId) REFERENCES QUIZ(id) ON DELETE CASCADE
);

-- Choix de réponse pour les questions QCM
CREATE TABLE CHOIX (
    id SERIAL PRIMARY KEY,
    questionId INTEGER NOT NULL,
    texte TEXT NOT NULL,
    estCorrect BOOLEAN NOT NULL,
    ordre INTEGER NOT NULL,
    FOREIGN KEY (questionId) REFERENCES QUESTION(id) ON DELETE CASCADE
);

-- ========== INDEX POUR PERFORMANCE ==========
CREATE INDEX idx_reservation_user ON RESERVATION(userId);
CREATE INDEX idx_reservation_machine ON RESERVATION(machineId);
CREATE INDEX idx_reservation_dates ON RESERVATION(dateDebut, dateFin);
CREATE INDEX idx_log_utilisation_machine ON LOG_UTILISATION(machineId);
CREATE INDEX idx_log_utilisation_user ON LOG_UTILISATION(userId);
CREATE INDEX idx_progression_user ON PROGRESSION(userId);
CREATE INDEX idx_progression_formation ON PROGRESSION(formationId);
CREATE INDEX idx_section_formation ON SECTION(formationId);
CREATE INDEX idx_quiz_formation ON QUIZ(formationId);
CREATE INDEX idx_question_quiz ON QUESTION(quizId);
CREATE INDEX idx_choix_question ON CHOIX(questionId);

