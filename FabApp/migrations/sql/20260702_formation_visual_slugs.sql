-- FABOS - slugs visuels demo pour les formations
-- SQL controle: remplit FORMATION.image seulement si aucune image n'est encore renseignee.

UPDATE FORMATION SET image = 'printer-3d' WHERE titre = 'Formation imprimante 3D' AND (image IS NULL OR image = '');
UPDATE FORMATION SET image = 'laser' WHERE titre = 'Formation découpe laser CO2' AND (image IS NULL OR image = '');
UPDATE FORMATION SET image = 'soldering' WHERE titre = 'Formation soudure électronique' AND (image IS NULL OR image = '');
UPDATE FORMATION SET image = 'vinyl' WHERE titre = 'Formation découpe vinyle' AND (image IS NULL OR image = '');
UPDATE FORMATION SET image = 'cnc' WHERE titre = 'Formation fraiseuse CNC' AND (image IS NULL OR image = '');
UPDATE FORMATION SET image = 'oscilloscope' WHERE titre = 'Formation oscilloscope numérique' AND (image IS NULL OR image = '');
UPDATE FORMATION SET image = 'embroidery' WHERE titre = 'Formation brodeuse numérique' AND (image IS NULL OR image = '');
