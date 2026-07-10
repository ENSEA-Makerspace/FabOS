-- FABOS - configuration des blocs de la page d'accueil
-- Script controle: a executer manuellement apres validation.

CREATE TABLE IF NOT EXISTS HOMEPAGE_SECTION_VISIBILITY (
    id INT AUTO_INCREMENT PRIMARY KEY,
    sectionKey VARCHAR(100) NOT NULL,
    label VARCHAR(150) NOT NULL,
    visibleAnonymous TINYINT(1) NOT NULL DEFAULT 0,
    visibleUser TINYINT(1) NOT NULL DEFAULT 0,
    visibleStaff TINYINT(1) NOT NULL DEFAULT 0,
    visibleAdmin TINYINT(1) NOT NULL DEFAULT 1,
    sortOrder INT NOT NULL DEFAULT 0,
    updatedAt DATETIME DEFAULT NULL,
    UNIQUE KEY uniq_homepage_section_visibility_key (sectionKey),
    INDEX idx_homepage_section_visibility_sort (sortOrder)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

INSERT INTO HOMEPAGE_SECTION_VISIBILITY (sectionKey, label, visibleAnonymous, visibleUser, visibleStaff, visibleAdmin, sortOrder, updatedAt)
SELECT 'opening_hours', 'Horaires d’ouverture', 1, 1, 1, 1, 10, NOW()
WHERE NOT EXISTS (SELECT 1 FROM HOMEPAGE_SECTION_VISIBILITY WHERE sectionKey = 'opening_hours');

INSERT INTO HOMEPAGE_SECTION_VISIBILITY (sectionKey, label, visibleAnonymous, visibleUser, visibleStaff, visibleAdmin, sortOrder, updatedAt)
SELECT 'how_it_works', 'Comment ça fonctionne ?', 1, 1, 1, 1, 20, NOW()
WHERE NOT EXISTS (SELECT 1 FROM HOMEPAGE_SECTION_VISIBILITY WHERE sectionKey = 'how_it_works');

INSERT INTO HOMEPAGE_SECTION_VISIBILITY (sectionKey, label, visibleAnonymous, visibleUser, visibleStaff, visibleAdmin, sortOrder, updatedAt)
SELECT 'fablab_stats', 'Statistiques FabLab', 0, 0, 1, 1, 30, NOW()
WHERE NOT EXISTS (SELECT 1 FROM HOMEPAGE_SECTION_VISIBILITY WHERE sectionKey = 'fablab_stats');

INSERT INTO HOMEPAGE_SECTION_VISIBILITY (sectionKey, label, visibleAnonymous, visibleUser, visibleStaff, visibleAdmin, sortOrder, updatedAt)
SELECT 'mini_leaderboard', 'Mini classement', 0, 1, 1, 1, 40, NOW()
WHERE NOT EXISTS (SELECT 1 FROM HOMEPAGE_SECTION_VISIBILITY WHERE sectionKey = 'mini_leaderboard');

INSERT INTO HOMEPAGE_SECTION_VISIBILITY (sectionKey, label, visibleAnonymous, visibleUser, visibleStaff, visibleAdmin, sortOrder, updatedAt)
SELECT 'featured_machines', 'Machines à découvrir', 1, 1, 1, 1, 50, NOW()
WHERE NOT EXISTS (SELECT 1 FROM HOMEPAGE_SECTION_VISIBILITY WHERE sectionKey = 'featured_machines');

INSERT INTO HOMEPAGE_SECTION_VISIBILITY (sectionKey, label, visibleAnonymous, visibleUser, visibleStaff, visibleAdmin, sortOrder, updatedAt)
SELECT 'latest_rfid_logs', 'Derniers passages RFID', 0, 0, 1, 1, 60, NOW()
WHERE NOT EXISTS (SELECT 1 FROM HOMEPAGE_SECTION_VISIBILITY WHERE sectionKey = 'latest_rfid_logs');

SELECT COUNT(*) AS homepage_section_visibility_count FROM HOMEPAGE_SECTION_VISIBILITY;
