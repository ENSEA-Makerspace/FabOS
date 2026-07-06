CREATE TABLE IF NOT EXISTS OPENING_HOUR (
    id INT AUTO_INCREMENT PRIMARY KEY,
    dayOfWeek TINYINT NOT NULL,
    label VARCHAR(20) NOT NULL,
    isClosed TINYINT(1) NOT NULL DEFAULT 0,
    openTime TIME NULL,
    closeTime TIME NULL,
    sortOrder INT NOT NULL DEFAULT 0,
    updatedAt DATETIME NULL,
    UNIQUE KEY uniq_opening_hour_day_of_week (dayOfWeek)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

INSERT INTO OPENING_HOUR (dayOfWeek, label, isClosed, openTime, closeTime, sortOrder, updatedAt)
SELECT 1, 'Lundi', 0, '08:00:00', '20:00:00', 1, NOW()
WHERE NOT EXISTS (SELECT 1 FROM OPENING_HOUR WHERE dayOfWeek = 1);

INSERT INTO OPENING_HOUR (dayOfWeek, label, isClosed, openTime, closeTime, sortOrder, updatedAt)
SELECT 2, 'Mardi', 0, '08:00:00', '20:00:00', 2, NOW()
WHERE NOT EXISTS (SELECT 1 FROM OPENING_HOUR WHERE dayOfWeek = 2);

INSERT INTO OPENING_HOUR (dayOfWeek, label, isClosed, openTime, closeTime, sortOrder, updatedAt)
SELECT 3, 'Mercredi', 0, '08:00:00', '20:00:00', 3, NOW()
WHERE NOT EXISTS (SELECT 1 FROM OPENING_HOUR WHERE dayOfWeek = 3);

INSERT INTO OPENING_HOUR (dayOfWeek, label, isClosed, openTime, closeTime, sortOrder, updatedAt)
SELECT 4, 'Jeudi', 0, '08:00:00', '20:00:00', 4, NOW()
WHERE NOT EXISTS (SELECT 1 FROM OPENING_HOUR WHERE dayOfWeek = 4);

INSERT INTO OPENING_HOUR (dayOfWeek, label, isClosed, openTime, closeTime, sortOrder, updatedAt)
SELECT 5, 'Vendredi', 0, '08:00:00', '22:00:00', 5, NOW()
WHERE NOT EXISTS (SELECT 1 FROM OPENING_HOUR WHERE dayOfWeek = 5);

INSERT INTO OPENING_HOUR (dayOfWeek, label, isClosed, openTime, closeTime, sortOrder, updatedAt)
SELECT 6, 'Samedi', 0, '09:00:00', '18:00:00', 6, NOW()
WHERE NOT EXISTS (SELECT 1 FROM OPENING_HOUR WHERE dayOfWeek = 6);

INSERT INTO OPENING_HOUR (dayOfWeek, label, isClosed, openTime, closeTime, sortOrder, updatedAt)
SELECT 7, 'Dimanche', 1, NULL, NULL, 7, NOW()
WHERE NOT EXISTS (SELECT 1 FROM OPENING_HOUR WHERE dayOfWeek = 7);

SELECT COUNT(*) AS opening_hour_count FROM OPENING_HOUR;
