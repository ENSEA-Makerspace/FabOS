<?php
namespace App\Identity;
final readonly class OidcProvider { public function __construct(public string $key, public string $label, public string $issuer, public string $clientId, public string $secretEnv, public array $scopes, public bool $enabled) {} }
