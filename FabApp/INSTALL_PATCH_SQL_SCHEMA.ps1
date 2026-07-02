# A lancer depuis la racine de ton projet Symfony, ou adapte $ProjectRoot.
$ProjectRoot = Get-Location

# Supprime les anciens fichiers d'entités/repositories qui ne correspondent plus au SQL historique.
$oldEntities = @(
  "src\Entity\MachineCategory.php",
  "src\Entity\TlseUser.php",
  "src\Entity\TrainingEnrollment.php",
  "src\Entity\UserBadge.php",
  "src\Entity\MachineUsageHistory.php",
  "src\Entity\Notification.php"
)
$oldRepositories = @(
  "src\Repository\MachineCategoryRepository.php",
  "src\Repository\TlseUserRepository.php",
  "src\Repository\TrainingEnrollmentRepository.php",
  "src\Repository\UserBadgeRepository.php",
  "src\Repository\MachineUsageHistoryRepository.php",
  "src\Repository\NotificationRepository.php"
)
foreach ($file in $oldEntities + $oldRepositories) {
  $path = Join-Path $ProjectRoot $file
  if (Test-Path $path) { Remove-Item $path -Force }
}

Write-Host "Anciens fichiers neutralisés. Copie maintenant le contenu du patch puis lance les commandes indiquées."
