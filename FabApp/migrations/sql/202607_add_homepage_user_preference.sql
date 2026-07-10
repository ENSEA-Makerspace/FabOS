-- FABOS - preferences utilisateur pour l'ordre des blocs de la page d'accueil
-- Script controle: a executer manuellement apres validation.

CREATE TABLE IF NOT EXISTS HOMEPAGE_USER_PREFERENCE (
    id INT AUTO_INCREMENT PRIMARY KEY,
    userId INT NOT NULL,
    sectionOrder TEXT NOT NULL,
    updatedAt DATETIME DEFAULT NULL,

    UNIQUE KEY uniq_homepage_user_preference_user (userId),
    INDEX idx_homepage_user_preference_user (userId),

    CONSTRAINT fk_homepage_user_preference_user
        FOREIGN KEY (userId)
        REFERENCES UTILISATEUR(id)
        ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

SELECT COUNT(*) AS homepage_user_preference_count FROM HOMEPAGE_USER_PREFERENCE;
