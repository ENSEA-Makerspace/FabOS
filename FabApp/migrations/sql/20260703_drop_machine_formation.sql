-- FABOS - suppression definitive de l'ancien modele MACHINE_FORMATION
-- Pre-conditions verifiees avant execution controlee :
-- - BADGE = 7
-- - FORMATION = 7 et aucune formation sans badge
-- - MACHINE = 9
-- - MACHINE_BADGE = 9 et chaque machine a au moins un badge requis
-- - MACHINE_FORMATION = 9 avant suppression
-- Ne touche a aucune autre table.

DROP TABLE IF EXISTS MACHINE_FORMATION;
