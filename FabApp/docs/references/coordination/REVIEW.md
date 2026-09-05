# Revue finale — Coordination, découverte et confiance

## Calendrier

- Proposer une lecture unifiée Machines, Espaces, Événements et Personnes, avec filtres par type et lieu à un clic.
- Toute entrée ouvre une fiche compacte puis sa fiche complète. Les couleurs complètent le texte : elles ne portent pas seules l’état.

## Rendez-vous et disponibilités

- La fiche personne doit commencer par le premier créneau disponible ; le membre choisit date puis créneau, et confirme en une seule étape.
- L’agent gère des plages simples, congés et demandes, plutôt qu’un tableau complexe.

## Groupes et badges

- Un groupe explique sa finalité, ses membres et les droits conférés avant toute édition. Archiver plutôt que supprimer lorsqu’il existe un historique.
- Le badge membre ne montre jamais son UID ni la granularité d’un journal de portes. Il affiche uniquement statut, périmètre humainement compréhensible et procédure de remplacement.

## Recherche et e-mail

- La recherche doit couvrir les objets publics et autorisés ; les résultats affichent type, contexte et destination claire. Avant saisie, proposer des raccourcis utiles.
- Séparer e-mails transactionnels obligatoires et préférences optionnelles. Écrire une phrase par notification ; le changement est visible avant l’enregistrement.

## Prompt d’implémentation

> Complète FabOS avec les parcours Coordination et confiance selon les maquettes `docs/references/coordination`. Réutilise exclusivement le shell, les filtres, les états, les formulaires et la charte `/admin/design` partagés. Implémente un calendrier filtrable, prise de rendez-vous, groupes, badge membre, recherche globale et préférences e-mail ; assure une cible détail claire dans chaque liste. N’expose ni UID, ni secret, ni historique détaillé de contrôle d’accès à un membre. Vérifie les droits sur chaque résultat de recherche et chaque action. Mesure les clics du parcours commun, ajoute tests et documentation, déploie sur Artemis puis commit par lot.
